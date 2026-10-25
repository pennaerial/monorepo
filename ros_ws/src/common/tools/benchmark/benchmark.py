#!/usr/bin/env python3
"""
benchmark.py

Polls psutil for all processes matching a ROS-related filter and tracks
per-process resource usage over time, so you can spot which nodes are
bottlenecking your pipeline.

Metrics tracked per process (averaged + max over the run):
  - CPU %          (psutil convention: 100% == 1 full core saturated)
  - RSS memory     (resident set size, MB)
  - Threads        (thread count -- spikes often mean callback/queue pileup)
  - IO read/write  (bytes/sec, when accessible -- disk/logging bottlenecks)
  - Open FDs       (file descriptors -- leaks/too many topic connections)
  - CPU% / thread  (rough "how hot is each thread running")

Usage:
  python3 ros_process_monitor.py                     # run until Ctrl+C
  python3 ros_process_monitor.py --duration 60        # run for 60s
  python3 ros_process_monitor.py --interval 0.5       # poll every 0.5s
  python3 ros_process_monitor.py --filter ros,gazebo  # custom match terms
  python3 ros_process_monitor.py --csv out.csv        # dump raw samples too
  python3 ros_process_monitor.py --top 15             # show top 15 in report

Notes:
  - CPU% is normalized to a single core (psutil default). A node showing
    150% is using 1.5 cores; on an 8-core box max possible is 800%.
  - Run with the same (or higher) privileges as your ROS nodes, or IO/FD
    stats may show as "n/a" for processes you don't own.
"""

import argparse
import csv
import os
import signal
import sys
import time

import psutil


def matches_filter(proc, terms):
    """Check process name + full cmdline against filter terms (case-insensitive)."""
    try:
        name = proc.name() or ""
        cmdline = " ".join(proc.cmdline())
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        return False
    haystack = (name + " " + cmdline).lower()
    return any(t in haystack for t in terms)


def process_labels(proc):
    """Build a full label and a "readable" command for a process.

    full_label keeps the entire command line (e.g. 'ros2 run pkg camera_node'),
    so launcher processes like 'ros2 run ...' / 'ros2 launch ...' are
    distinguishable. readable_cmd trims long interpreter/install-path prefixes
    down to their basename (e.g. '/opt/ros/jazzy/bin/ros2' -> 'ros2') so the
    important part -- the binary + its args -- stays readable inline, e.g. in
    a markdown table cell.
    """
    try:
        cmdline = proc.cmdline()
        name = proc.name()
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        fallback = f"pid-{proc.pid}"
        return fallback, fallback
    full_cmd = " ".join(cmdline) if cmdline else name
    full_label = f"{full_cmd} [pid {proc.pid}]"
    tokens = cmdline or [name]
    readable_cmd = " ".join(os.path.basename(tok) if tok.startswith("/") else tok for tok in tokens)
    return full_label, readable_cmd or name


class ProcTracker:
    """Tracks running stats for one process across polls."""

    def __init__(self, proc):
        self.proc = proc
        self.pid = proc.pid
        self.label, self.cmd = process_labels(proc)
        self.samples = 0
        self.cpu_sum = 0.0
        self.cpu_max = 0.0
        self.mem_sum = 0.0
        self.mem_max = 0.0
        self.threads_sum = 0
        self.threads_max = 0
        self.fds_sum = 0
        self.fds_max = 0
        self._last_io = None
        self._last_time = None
        self.io_read_bps = []
        self.io_write_bps = []

    def poll(self):
        try:
            with self.proc.oneshot():
                cpu = self.proc.cpu_percent(interval=None)
                mem_mb = self.proc.memory_info().rss / (1024 * 1024)
                threads = self.proc.num_threads()
                try:
                    fds = self.proc.num_fds()
                except (psutil.AccessDenied, AttributeError):
                    fds = None
                try:
                    io = self.proc.io_counters()
                except (psutil.AccessDenied, NotImplementedError):
                    io = None
        except (psutil.NoSuchProcess, psutil.ZombieProcess):
            return False

        now = time.time()
        self.samples += 1
        self.cpu_sum += cpu
        self.cpu_max = max(self.cpu_max, cpu)
        self.mem_sum += mem_mb
        self.mem_max = max(self.mem_max, mem_mb)
        self.threads_sum += threads
        self.threads_max = max(self.threads_max, threads)
        if fds is not None:
            self.fds_sum += fds
            self.fds_max = max(self.fds_max, fds)

        if io is not None and self._last_io is not None and self._last_time is not None:
            dt = max(now - self._last_time, 1e-6)
            self.io_read_bps.append((io.read_bytes - self._last_io.read_bytes) / dt)
            self.io_write_bps.append((io.write_bytes - self._last_io.write_bytes) / dt)
        self._last_io = io
        self._last_time = now

        self._last_cpu = cpu
        self._last_mem = mem_mb
        self._last_threads = threads
        self._last_fds = fds
        return True

    def row(self):
        avg_cpu = self.cpu_sum / self.samples if self.samples else 0
        avg_mem = self.mem_sum / self.samples if self.samples else 0
        avg_threads = self.threads_sum / self.samples if self.samples else 0
        avg_fds = self.fds_sum / self.samples if self.samples else None
        avg_read = sum(self.io_read_bps) / len(self.io_read_bps) if self.io_read_bps else None
        avg_write = sum(self.io_write_bps) / len(self.io_write_bps) if self.io_write_bps else None
        cpu_per_thread = avg_cpu / avg_threads if avg_threads else 0
        return {
            "label": self.label,
            "cmd": self.cmd,
            "pid": self.pid,
            "samples": self.samples,
            "avg_cpu": avg_cpu,
            "max_cpu": self.cpu_max,
            "avg_mem_mb": avg_mem,
            "max_mem_mb": self.mem_max,
            "avg_threads": avg_threads,
            "max_threads": self.threads_max,
            "avg_fds": avg_fds,
            "max_fds": self.fds_max if self.fds_sum else None,
            "avg_read_bps": avg_read,
            "avg_write_bps": avg_write,
            "cpu_per_thread": cpu_per_thread,
        }


def fmt_bps(v):
    if v is None:
        return "n/a"
    for unit in ["B/s", "KB/s", "MB/s", "GB/s"]:
        if abs(v) < 1024:
            return f"{v:.1f}{unit}"
        v /= 1024
    return f"{v:.1f}TB/s"


def truncate(s, width):
    return s if len(s) <= width else s[: width - 1] + "…"


PROC_W = 26
WIDTH = 90


def print_report(trackers, top_n):
    rows = [t.row() for t in trackers.values() if t.row()["samples"] > 0]
    if not rows:
        print("No matching ROS processes were observed.")
        return

    rows.sort(key=lambda r: r["avg_cpu"], reverse=True)
    rows = rows[:top_n]

    print("\n" + "=" * WIDTH)
    print(f"ROS PROCESS RESOURCE REPORT (top {len(rows)} by avg CPU%)".center(WIDTH))
    print("=" * WIDTH)
    print(
        f"{'#':>3} {'Process':<{PROC_W}} {'PID':>6} {'CPU avg/max':>13} "
        f"{'Mem avg/max MB':>15} {'Thr':>5} {'FDs':>5} {'IO R/W':>16} {'CPU/Thr':>7}"
    )
    print("-" * WIDTH)

    for i, r in enumerate(rows, start=1):
        fds_str = f"{r['avg_fds']:.0f}" if r["avg_fds"] is not None else "n/a"
        cpu_str = f"{r['avg_cpu']:.1f}/{r['max_cpu']:.1f}%"
        mem_str = f"{r['avg_mem_mb']:.0f}/{r['max_mem_mb']:.0f}"
        io_str = f"R{fmt_bps(r['avg_read_bps'])}/W{fmt_bps(r['avg_write_bps'])}"
        print(
            f"{i:>3} {truncate(r['cmd'], PROC_W):<{PROC_W}} {r['pid']:>6} {cpu_str:>13} "
            f"{mem_str:>15} {r['avg_threads']:>5.1f} {fds_str:>5} {io_str:>16} {r['cpu_per_thread']:>6.1f}%"
        )

    print("-" * WIDTH)
    print("\nLikely bottleneck signals:")
    top_cpu = rows[0]
    print(f"  * Highest avg CPU : {truncate(top_cpu['cmd'], 60)} ({top_cpu['avg_cpu']:.1f}%)")
    top_mem = max(rows, key=lambda r: r["avg_mem_mb"])
    print(f"  * Highest avg RAM : {truncate(top_mem['cmd'], 60)} ({top_mem['avg_mem_mb']:.1f}MB)")
    spiky = max(rows, key=lambda r: r["max_cpu"] - r["avg_cpu"])
    if spiky["max_cpu"] - spiky["avg_cpu"] > 20:
        print(
            f"  * Bursty CPU      : {truncate(spiky['cmd'], 60)} "
            f"(avg {spiky['avg_cpu']:.1f}% vs max {spiky['max_cpu']:.1f}%) -> possible stalls/callbacks"
        )
    threadiest = max(rows, key=lambda r: r["avg_threads"])
    print(
        f"  * Most threads    : {truncate(threadiest['cmd'], 60)} ({threadiest['avg_threads']:.1f} avg)"
    )
    io_rows = [r for r in rows if r["avg_write_bps"] is not None]
    if io_rows:
        top_io = max(io_rows, key=lambda r: (r["avg_write_bps"] or 0) + (r["avg_read_bps"] or 0))
        print(
            f"  * Heaviest IO     : {truncate(top_io['cmd'], 60)} "
            f"(R {fmt_bps(top_io['avg_read_bps'])}, W {fmt_bps(top_io['avg_write_bps'])})"
        )
    print("=" * WIDTH)


def write_csv(trackers, path):
    rows = [t.row() for t in trackers.values() if t.row()["samples"] > 0]
    if not rows:
        return
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nRaw per-process summary written to: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Poll psutil for ROS processes and report resource usage."
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between polls (default: 1.0)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Total seconds to run (default: run until Ctrl+C)",
    )
    parser.add_argument(
        "--filter",
        type=str,
        default="ros",
        help="Comma-separated substrings to match in name/cmdline (default: 'ros')",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=20,
        help="Number of processes to show in the report (default: 20)",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Optional path to dump the final summary as CSV",
    )
    args = parser.parse_args()

    terms = [t.strip().lower() for t in args.filter.split(",") if t.strip()]
    print(f"Polling every {args.interval}s for processes matching: {terms}")
    print("Press Ctrl+C to stop and print the report.\n")

    trackers = {}
    stop = {"flag": False}

    def handle_sigint(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, handle_sigint)

    start = time.time()
    poll_count = 0
    try:
        while not stop["flag"]:
            poll_count += 1
            seen_pids = set()
            new_pids = set()
            for proc in psutil.process_iter(["pid", "name"]):
                if proc.pid not in trackers:
                    if matches_filter(proc, terms):
                        try:
                            proc.cpu_percent(interval=None)  # prime the counter
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            continue
                        trackers[proc.pid] = ProcTracker(proc)
                        new_pids.add(proc.pid)
                if proc.pid in trackers:
                    seen_pids.add(proc.pid)

            for pid in seen_pids:
                if pid in new_pids:
                    # Skip this cycle: cpu_percent() was just primed above, so
                    # polling it now would measure CPU% over a near-zero
                    # interval, which is noisy/meaningless (psutil footgun).
                    # Wait a full poll interval before taking its first sample.
                    continue
                trackers[pid].poll()

            live_count = sum(1 for t in trackers.values() if t.samples > 0)
            elapsed = time.time() - start
            sys.stdout.write(
                f"\r[{elapsed:6.1f}s] polling... tracking {live_count} process(es), {poll_count} sample(s)   "
            )
            sys.stdout.flush()

            if args.duration is not None and elapsed >= args.duration:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass

    print()  # newline after the \r progress line
    print_report(trackers, args.top)
    if args.csv:
        write_csv(trackers, args.csv)


if __name__ == "__main__":
    main()
