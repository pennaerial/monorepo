from __future__ import annotations

import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass(slots=True)
class RuntimeConfig:
    pi_user: str
    pi_host: str
    remote_dir: str
    ssh_key: str
    ssh_pass: str
    github_repo: str
    github_token: str
    hotspot_name: str

    @classmethod
    def from_env(cls, base_dir: Path) -> "RuntimeConfig":
        cfg = cls(
            pi_user=os.environ.get("PI_USER", "penn"),
            pi_host=os.environ.get("PI_HOST", "penn-desktop.local"),
            remote_dir=os.environ.get(
                "REMOTE_DIR", "/home/penn/monorepo/controls/sae_2025_ws"
            ),
            ssh_key=os.environ.get("SSH_KEY", ""),
            ssh_pass=os.environ.get("SSH_PASS", ""),
            github_repo=os.environ.get("GITHUB_REPO", ""),
            github_token=os.environ.get("GITHUB_TOKEN", ""),
            hotspot_name=os.environ.get("HOTSPOT_CON_NAME", "penn-desktop"),
        )
        if not cfg.github_repo:
            cfg.github_repo = _detect_github_repo(base_dir)
        return cfg

    def ssh_target(self) -> str:
        return f"{self.pi_user}@{self.pi_host}"

    def mission_paths(self) -> dict[str, str]:
        return {
            "log": f"{self.remote_dir}/.mission_main_launch.log",
            "pid": f"{self.remote_dir}/.mission_main_launch.pid",
            "pgid": f"{self.remote_dir}/.mission_main_launch.pgid",
            "launch_params": f"{self.remote_dir}/src/uav/launch/launch_params.yaml",
            "missions_dir": f"{self.remote_dir}/src/uav/uav/missions",
        }

    def to_safe_dict(self) -> dict[str, str]:
        return {
            "pi_user": self.pi_user,
            "pi_host": self.pi_host,
            "remote_dir": self.remote_dir,
            "ssh_key": self.ssh_key,
            "ssh_pass": "••••" if self.ssh_pass else "",
            "github_repo": self.github_repo,
            "hotspot_name": self.hotspot_name,
        }

    def update_from_form(self, updates: dict[str, str | None]) -> dict[str, str]:
        changed: dict[str, str] = {}

        def _set(field: str, value: str | None):
            if value is None:
                return
            setattr(self, field, value)
            changed[field] = value

        _set("pi_user", updates.get("pi_user"))
        _set("pi_host", updates.get("pi_host"))
        _set("remote_dir", updates.get("remote_dir"))
        _set("ssh_key", updates.get("ssh_key"))

        ssh_pass = updates.get("ssh_pass")
        if ssh_pass is not None and ssh_pass != "••••":
            self.ssh_pass = ssh_pass
            changed["ssh_pass"] = "(set)"

        _set("github_repo", updates.get("github_repo"))
        _set("hotspot_name", updates.get("hotspot_name"))

        return changed


def _detect_github_repo(base_dir: Path) -> str:
    try:
        result = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            capture_output=True,
            text=True,
            cwd=base_dir,
            timeout=3,
        )
        match = re.search(r"github\.com[:/]([^/]+/[^/.]+)", result.stdout)
        if match:
            return match.group(1).strip()
    except Exception:
        pass
    return ""
