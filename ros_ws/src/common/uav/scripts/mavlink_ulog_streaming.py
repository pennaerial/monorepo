#!/usr/bin/env python3

"""
Stream ULog data over MAVLink.

@author: Beat Kueng (beat-kueng@gmx.net)
"""

from __future__ import print_function

import datetime
import os
import sys
from timeit import default_timer as timer

os.environ["MAVLINK20"] = "1"  # The commands require mavlink 2
import signal
from argparse import ArgumentParser

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)


STOP_TIMEOUT = 2.0  # Finish before ROS launch's default 5-second SIGTERM escalation.


class LoggingCompleted(Exception):
    pass


class MavlinkLogStreaming:
    """Streams log data via MAVLink.
    Assumptions:
    - the sender only sends one acked message at a time
    - the data is in the ULog format"""

    def __init__(self, debug=0):
        self._debug = debug
        self.mav = None
        self.file = None
        self.got_ulog_header = False
        self.got_header_section = False
        self.ulog_message = []
        self.start_time = 0
        self.last_sequence = -1
        self.logging_started = False
        self.num_dropouts = 0
        self.shutdown_signal = None

    def connect(self, portname, baudrate, output_filename):
        """Wait for PX4, allowing shutdown before a heartbeat arrives."""
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        # PX4 can forward other components' heartbeats over this connection.
        # A GCS heartbeat satisfies wait_heartbeat() without setting a vehicle target.
        while self.shutdown_signal is None:
            heartbeat = self.mav.wait_heartbeat(timeout=0.1)
            if (
                heartbeat is not None
                and heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
                # Reject zero IDs so commands target PX4, not a broadcast.
                and heartbeat.get_srcSystem() != 0
                and heartbeat.get_srcComponent() != 0
            ):
                self.target_system = heartbeat.get_srcSystem()
                self.target_component = heartbeat.get_srcComponent()
                self.mav.target_system = self.target_system
                self.mav.target_component = self.target_component
                break
        else:
            return False
        print(f"Connected to PX4: system {self.target_system}, component {self.target_component}")
        self.debug("Locked serial device\n")

        self.file = open(output_filename, "wb")
        self.start_time = timer()
        return True

    def close(self):
        try:
            if self.file is not None:
                self.file.close()  # Flush buffered writes; omit any incomplete ULog record.
        finally:
            if self.mav is not None:
                self.mav.close()

    def debug(self, s, level=1):
        """write some debug text"""
        if self._debug >= level:
            print(s)

    def start_log(self):
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_LOGGING_START,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def stop_log(self):
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_LOGGING_STOP,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def _handle_signal(self, sig, frame):
        if self.shutdown_signal != signal.SIGTERM:
            self.shutdown_signal = sig

    def read_messages(self):
        """main loop reading messages"""
        measure_time_start = timer()
        measured_data = 0
        deadline = None

        while self.shutdown_signal != signal.SIGTERM:
            if self.shutdown_signal == signal.SIGINT and deadline is None:
                deadline = timer() + STOP_TIMEOUT
                print("\nStopping log...")
                self.stop_log()
                # Keep reading until PX4 acknowledges or the shutdown deadline expires.

            if deadline is not None and timer() >= deadline:
                print("\nStop not confirmed; closing local log.")
                return

            m, first_msg_start, num_drops = self.read_message()
            if m is not None:
                self.process_streamed_ulog_data(m, first_msg_start, num_drops)

                # status output
                if self.logging_started:
                    measured_data += len(m)
                    measure_time_cur = timer()
                    dt = measure_time_cur - measure_time_start
                    if dt > 1:
                        sys.stdout.write(
                            "\rData Rate: {:0.1f} KB/s  Drops: {:} \033[K".format(
                                measured_data / dt / 1024, self.num_dropouts
                            )
                        )
                        sys.stdout.flush()
                        measure_time_start = measure_time_cur
                        measured_data = 0

            if deadline is None and not self.logging_started and timer() - self.start_time > 15:
                raise Exception("Start timed out. Is the logger running in MAVLink mode?")

    def read_message(self):
        """read a single mavlink message, handle ACK & return a tuple of (data, first
        message start, num dropouts)"""
        m = self.mav.recv_match(
            type=["LOGGING_DATA_ACKED", "LOGGING_DATA", "COMMAND_ACK"], blocking=True, timeout=0.05
        )
        if m is not None:
            self.debug(m, 3)

            if m.get_type() == "COMMAND_ACK":
                # acknowledgement from px4 of command that we sent
                if (
                    m.command == mavutil.mavlink.MAV_CMD_LOGGING_START
                    and not self.got_header_section
                ):
                    # we sent a logging command but header hasn't started yet
                    if m.result == 0:
                        # logging start command accepted by px4
                        self.logging_started = True
                        print("Logging started. Waiting for Header...")
                    else:
                        # logging start command not accepted
                        raise Exception("Logging start failed", m.result)
                elif (
                    m.command == mavutil.mavlink.MAV_CMD_LOGGING_STOP
                    and m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                ):
                    # we sent stop logging command and it was accepted
                    raise LoggingCompleted()
                return None, 0, 0

            # m is either 'LOGGING_DATA_ACKED' or 'LOGGING_DATA':
            is_newer, num_drops = self.check_sequence(m.sequence)

            # return an ack, even we already sent it for the same sequence,
            # because the ack could have been dropped
            if m.get_type() == "LOGGING_DATA_ACKED":
                self.mav.mav.logging_ack_send(self.target_system, self.target_component, m.sequence)

            if is_newer:
                if num_drops > 0:
                    self.num_dropouts += num_drops

                if m.get_type() == "LOGGING_DATA":
                    if not self.got_header_section:
                        print(
                            "Header received in {:0.2f}s (size: {:.1f} KB)".format(
                                timer() - self.start_time, self.file.tell() / 1024
                            )
                        )
                        self.logging_started = True
                        self.got_header_section = True
                self.last_sequence = m.sequence
                return m.data[: m.length], m.first_message_offset, num_drops

            else:
                self.debug("dup/reordered message " + str(m.sequence))

        return None, 0, 0

    def check_sequence(self, seq):
        """check if a sequence is newer than the previously received one & if
        there were dropped messages between the last and this"""
        if self.last_sequence == -1:
            return True, 0
        if seq == self.last_sequence:  # duplicate
            return False, 0
        if seq > self.last_sequence:
            # account for wrap-arounds, sequence is 2 bytes
            if seq - self.last_sequence > (1 << 15):  # assume reordered
                return False, 0
            return True, seq - self.last_sequence - 1
        else:
            if self.last_sequence - seq > (1 << 15):
                return True, (1 << 16) - self.last_sequence - 1 + seq
            return False, 0

    def process_streamed_ulog_data(self, data, first_msg_start, num_drops):
        """write streamed data to a file"""
        if not self.got_ulog_header:  # the first 16 bytes need special treatment
            if len(data) < 16:  # that's never the case anyway
                raise Exception("first received message too short")
            self.file.write(bytearray(data[0:16]))
            data = data[16:]
            self.got_ulog_header = True

        if self.got_header_section and num_drops > 0:
            if num_drops > 25:
                num_drops = 25
            # write a dropout message. We don't really know the actual duration,
            # so just use the number of drops * 10 ms
            self.file.write(bytearray([2, 0, 79, num_drops * 10, 0]))

        if num_drops > 0:
            self.write_ulog_messages(self.ulog_message)
            self.ulog_message = []
            if first_msg_start == 255:
                return  # no useful information in this message: drop it
            data = data[first_msg_start:]
            first_msg_start = 0

        if first_msg_start == 255 and len(self.ulog_message) > 0:
            self.ulog_message.extend(data)
            return

        if len(self.ulog_message) > 0:
            self.file.write(bytearray(self.ulog_message + data[:first_msg_start]))
            self.ulog_message = []

        data = self.write_ulog_messages(data[first_msg_start:])
        self.ulog_message = data  # store the rest for the next message

    def write_ulog_messages(self, data):
        """write ulog data w/o integrity checking, assuming data starts with a
        valid ulog message. returns the remaining data at the end."""
        while len(data) > 2:
            message_length = data[0] + data[1] * 256 + 3  # 3=ULog msg header
            if message_length > len(data):
                break
            self.file.write(bytearray(data[:message_length]))
            data = data[message_length:]
        return data


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "port",
        metavar="PORT",
        nargs="?",
        default=None,
        help="Mavlink port name: serial: DEVICE[,BAUD], udp: IP:PORT, tcp: tcp:IP:PORT. Eg: \
/dev/ttyUSB0 or 0.0.0.0:14550. Auto-detect serial if not given.",
    )
    parser.add_argument(
        "--baudrate",
        "-b",
        dest="baudrate",
        type=int,
        help="Mavlink port baud rate (default=115200)",
        default=115200,
    )
    parser.add_argument(
        "--output", "-o", dest="output", default=".", help="output file or directory (default=CWD)"
    )
    args = parser.parse_args()

    if os.path.isdir(args.output):
        filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S.ulg")
        filename = os.path.join(args.output, filename)
    else:
        filename = args.output
    print("Output file name: {:}".format(filename))

    if args.port is None:
        serial_list = mavutil.auto_detect_serial(
            preferred_list=[
                "*FTDI*",
                "*Arduino_Mega_2560*",
                "*3D_Robotics*",
                "*USB_to_UART*",
                "*PX4*",
                "*FMU*",
            ]
        )

        if len(serial_list) == 0:
            print("Error: no serial connection found")
            return

        if len(serial_list) > 1:
            print("Auto-detected serial ports are:")
            for port in serial_list:
                print(" {:}".format(port))
        print("Using port {:}".format(serial_list[0]))
        args.port = serial_list[0].device

    print("Connecting to MAVLINK...")
    stream = MavlinkLogStreaming()
    signals = (signal.SIGINT, signal.SIGTERM)
    old_handlers = {sig: signal.signal(sig, stream._handle_signal) for sig in signals}
    try:
        if not stream.connect(args.port, args.baudrate, filename) or stream.shutdown_signal:
            return
        print("Starting log...")
        stream.start_log()
        stream.read_messages()
    except KeyboardInterrupt:
        print("Aborting")
    except LoggingCompleted:
        print("Done")
    finally:
        try:
            stream.close()
        finally:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    main()
