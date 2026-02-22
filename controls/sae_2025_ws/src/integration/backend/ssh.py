from __future__ import annotations

import asyncio
import shlex
import shutil
import subprocess
from dataclasses import dataclass

from .config import RuntimeConfig


@dataclass(slots=True)
class SSHResult:
    returncode: int
    stdout: str
    stderr: str


class SSHExecutor:
    def __init__(self, config: RuntimeConfig):
        self.config = config

    @staticmethod
    def q(value: str) -> str:
        return shlex.quote(value)

    def ssh_opts(self) -> list[str]:
        opts = ["-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=3"]
        if self.config.ssh_key:
            opts += ["-i", self.config.ssh_key]
        if not self.config.ssh_pass:
            opts += ["-o", "BatchMode=yes", "-o", "NumberOfPasswordPrompts=0"]
        return opts

    def build_ssh_cmd(self, command: str, force_tty: bool = False) -> list[str]:
        cmd = ["ssh"]
        if force_tty:
            cmd.append("-tt")
        cmd += self.ssh_opts() + [self.config.ssh_target(), command]
        if self.config.ssh_pass:
            if not shutil.which("sshpass"):
                raise RuntimeError(
                    "SSH password auth is configured, but `sshpass` is not installed. "
                    "Install sshpass or clear SSH password in Settings and use SSH key auth."
                )
            cmd = ["sshpass", "-p", self.config.ssh_pass] + cmd
        return cmd

    def run_sync(self, command: str, timeout: int = 15) -> subprocess.CompletedProcess:
        cmd = self.build_ssh_cmd(command)
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)

    def scp_sync(
        self, local_path: str, remote_path: str, timeout: int = 300
    ) -> subprocess.CompletedProcess:
        remote_spec = f"{self.config.ssh_target()}:{self.q(remote_path)}"
        cmd = ["scp"] + self.ssh_opts() + [local_path, remote_spec]
        if self.config.ssh_pass:
            if not shutil.which("sshpass"):
                raise RuntimeError(
                    "SSH password auth is configured, but `sshpass` is not installed. "
                    "Install sshpass or clear SSH password in Settings and use SSH key auth."
                )
            cmd = ["sshpass", "-p", self.config.ssh_pass] + cmd
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)

    async def run(self, command: str, timeout: int = 15) -> subprocess.CompletedProcess:
        return await asyncio.to_thread(self.run_sync, command, timeout)

    async def scp(
        self, local_path: str, remote_path: str, timeout: int = 300
    ) -> subprocess.CompletedProcess:
        return await asyncio.to_thread(self.scp_sync, local_path, remote_path, timeout)

    def friendly_error(self, stderr: str) -> str:
        raw = (stderr or "").strip()
        lower = raw.lower()
        target = self.config.ssh_target()

        if any(
            s in lower
            for s in (
                "could not resolve hostname",
                "name or service not known",
                "no address associated with hostname",
                "nodename nor servname provided",
                "temporary failure in name resolution",
            )
        ):
            return (
                f"Cannot find {target}. Connect to the Pi WiFi and make sure "
                "Pi host/user in Settings are correct."
            )

        if any(
            s in lower
            for s in (
                "no route to host",
                "connection refused",
                "network is unreachable",
                "operation timed out",
                "connection timed out",
            )
        ):
            return (
                f"Cannot reach {target}. Connect to the Pi WiFi and make sure "
                "you are SSHing into the correct Pi host."
            )

        if any(
            s in lower
            for s in (
                "permission denied",
                "authentication failed",
                "auth fail",
                "access denied",
            )
        ):
            if self.config.ssh_pass:
                return (
                    f"SSH reached {target}, but the configured password was rejected. "
                    "Enter the correct SSH password in Settings."
                )
            return (
                f"SSH reached {target}, but password authentication is required and no SSH password is set. "
                "Enter the Pi SSH password in Settings."
            )

        if "host key verification failed" in lower:
            return (
                f"SSH host key verification failed for {target}. "
                "Clear the stale known_hosts entry for this host and retry."
            )

        if "ssh password auth is configured, but `sshpass` is not installed" in lower:
            return raw

        return raw or (
            f"SSH failed for {target}. Connect to the Pi WiFi and verify SSH target/user/password in Settings."
        )

    def friendly_timeout(self) -> str:
        target = self.config.ssh_target()
        return (
            f"SSH to {target} timed out. Connect to the Pi WiFi and make sure "
            "you are SSHing into the correct Pi host."
        )

    def format_remote_error(self, raw_error: str, prefix: str) -> str:
        raw = (raw_error or "").strip()
        friendly = self.friendly_error(raw)
        if friendly != raw or not raw:
            return friendly
        return f"{prefix}: {raw}"
