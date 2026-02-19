from __future__ import annotations

import os
import re
import tempfile
from pathlib import Path

from ..context import AppContext

_ARTIFACT_NAME_RE = re.compile(r"^[A-Za-z0-9._-]+$")


def _require_httpx():
    try:
        import httpx
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing Python dependency `httpx`. "
            "If using conda/pip, run: pip install httpx. "
            "If using uv, rerun with `uv run app.py`."
        ) from exc
    return httpx


def _github_headers(ctx: AppContext) -> dict:
    headers = {"Accept": "application/vnd.github+json"}
    if ctx.config.github_token:
        headers["Authorization"] = f"Bearer {ctx.config.github_token}"
    return headers


def sanitize_artifact_name(filename: str) -> str:
    name = Path(filename or "").name
    if not name:
        raise ValueError("Artifact filename is empty.")
    if not _ARTIFACT_NAME_RE.match(name):
        raise ValueError(
            "Artifact filename contains unsupported characters. "
            "Use letters, numbers, dots, underscores, and dashes only."
        )
    return name


async def _copy_artifact_to_pi(
    ctx: AppContext, local_path: str, artifact_name: str
) -> None:
    mkdir_result = await ctx.ssh.run(
        f"mkdir -p {ctx.ssh.q(ctx.config.remote_dir)}", timeout=10
    )
    if mkdir_result.returncode != 0:
        raise RuntimeError(
            ctx.ssh.format_remote_error(
                mkdir_result.stderr, "Prepare remote directory failed"
            )
        )

    remote_path = f"{ctx.config.remote_dir}/{artifact_name}"
    copy_result = await ctx.ssh.scp(local_path, remote_path, timeout=300)
    if copy_result.returncode != 0:
        raise RuntimeError(
            ctx.ssh.format_remote_error(copy_result.stderr, "SCP failed")
        )


async def _extract_artifact_on_pi(ctx: AppContext, artifact_name: str) -> None:
    cmd = f"""
        set -e
        cd {ctx.ssh.q(ctx.config.remote_dir)}
        if [ -d install ]; then
            rm -rf install.bak src.bak
            mv install install.bak 2>/dev/null || true
            mv src src.bak 2>/dev/null || true
        fi
        tar -xzf {ctx.ssh.q(artifact_name)}
        rm -f {ctx.ssh.q(artifact_name)}
        chmod -R +x install/
    """
    extract_result = await ctx.ssh.run(cmd, timeout=120)
    if extract_result.returncode != 0:
        raise RuntimeError(
            ctx.ssh.format_remote_error(extract_result.stderr, "Extract failed")
        )


async def current_build(ctx: AppContext) -> dict:
    try:
        build_info_path = f"{ctx.config.remote_dir}/install/BUILD_INFO.txt"
        install_dir = f"{ctx.config.remote_dir}/install"

        result = await ctx.ssh.run(
            f"cat {ctx.ssh.q(build_info_path)} 2>/dev/null", timeout=10
        )
        if result.returncode == 0 and result.stdout.strip():
            return {"success": True, "installed": True, "info": result.stdout.strip()}

        result = await ctx.ssh.run(
            f"test -d {ctx.ssh.q(install_dir)} && echo yes || echo no", timeout=10
        )
        if result.returncode != 0:
            return {
                "success": True,
                "installed": False,
                "info": ctx.ssh.friendly_error(result.stderr),
            }

        if result.stdout.strip() == "yes":
            return {
                "success": True,
                "installed": True,
                "info": "Build installed (no BUILD_INFO.txt)",
            }

        return {"success": True, "installed": False, "info": "No build deployed"}
    except Exception as exc:
        return {
            "success": True,
            "installed": False,
            "info": ctx.ssh.friendly_error(str(exc)),
        }


async def upload_build(ctx: AppContext, filename: str, file_bytes: bytes) -> dict:
    tmp_path = ""
    try:
        artifact_name = sanitize_artifact_name(filename)
        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            tmp.write(file_bytes)
            tmp_path = tmp.name

        await _copy_artifact_to_pi(ctx, tmp_path, artifact_name)
        await _extract_artifact_on_pi(ctx, artifact_name)

        return {"success": True, "output": f"Deployed {artifact_name} to Pi"}
    except RuntimeError as exc:
        return {"success": False, "error": str(exc)}
    except Exception as exc:
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


async def list_builds(ctx: AppContext) -> dict:
    if not ctx.config.github_repo:
        return {"success": False, "error": "GITHUB_REPO not configured", "builds": []}
    try:
        httpx = _require_httpx()
        async with httpx.AsyncClient(timeout=15) as client:
            resp = await client.get(
                f"https://api.github.com/repos/{ctx.config.github_repo}/releases",
                headers=_github_headers(ctx),
            )
            resp.raise_for_status()
            releases = resp.json()

        if isinstance(releases, dict) and "message" in releases:
            return {"success": False, "error": releases["message"], "builds": []}

        builds = []
        for rel in releases:
            if rel.get("tag_name", "").startswith("build-"):
                assets = rel.get("assets", [])
                builds.append(
                    {
                        "tag": rel["tag_name"],
                        "sha": rel["tag_name"].replace("build-", ""),
                        "name": rel.get("name", rel["tag_name"]),
                        "date": rel.get("published_at", "")[:10],
                        "download_url": assets[0]["browser_download_url"]
                        if assets
                        else None,
                        "size_mb": round(assets[0]["size"] / 1_000_000, 1)
                        if assets
                        else None,
                    }
                )
        return {"success": True, "builds": builds[:20]}
    except Exception as exc:
        return {"success": False, "error": str(exc), "builds": []}


async def download_build(ctx: AppContext, tag: str) -> dict:
    if not ctx.config.github_repo:
        return {"success": False, "error": "GITHUB_REPO not configured"}

    tmp_path = ""
    try:
        httpx = _require_httpx()
        async with httpx.AsyncClient(timeout=15) as client:
            resp = await client.get(
                f"https://api.github.com/repos/{ctx.config.github_repo}/releases/tags/{tag}",
                headers=_github_headers(ctx),
            )
            resp.raise_for_status()
            release = resp.json()

        assets = release.get("assets", [])
        if not assets:
            return {"success": False, "error": f"No assets found for {tag}"}

        download_url = assets[0]["browser_download_url"]
        artifact_name = sanitize_artifact_name(assets[0]["name"])

        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            tmp_path = tmp.name

        async with httpx.AsyncClient(timeout=300, follow_redirects=True) as client:
            async with client.stream(
                "GET", download_url, headers=_github_headers(ctx)
            ) as resp:
                resp.raise_for_status()
                with open(tmp_path, "wb") as out:
                    async for chunk in resp.aiter_bytes():
                        out.write(chunk)

        await _copy_artifact_to_pi(ctx, tmp_path, artifact_name)
        await _extract_artifact_on_pi(ctx, artifact_name)
        return {"success": True, "output": f"Downloaded {tag} and deployed to Pi"}
    except RuntimeError as exc:
        return {"success": False, "error": str(exc)}
    except Exception as exc:
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


async def rollback_build(ctx: AppContext) -> dict:
    try:
        rollback_cmd = f"""
            cd {ctx.ssh.q(ctx.config.remote_dir)}
            if [ ! -d install.bak ]; then
                echo "NO_BACKUP"
                exit 1
            fi
            if [ -d install ]; then
                mv install install.tmp
                mv install.bak install
                mv install.tmp install.bak
            else
                mv install.bak install
            fi
        """
        result = await ctx.ssh.run(rollback_cmd, timeout=30)
        if result.returncode != 0:
            if "NO_BACKUP" in result.stdout:
                return {"success": False, "error": "No backup build to rollback to"}
            return {"success": False, "error": ctx.ssh.friendly_error(result.stderr)}

        return {"success": True, "output": "Rolled back to previous build"}
    except Exception as exc:
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}
