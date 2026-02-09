from fastapi import FastAPI, Form
from fastapi.middleware.cors import CORSMiddleware
import subprocess
import os

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def get_git_root():
    """Dynamically get the git repository root"""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return os.getcwd()


REPO_PATH = get_git_root()
APP_PATH = os.path.join(REPO_PATH, "controls/sae_2025_ws/src/integration")


def get_git_commits(branch, limit=5):
    """Fetch recent git commits from a specific branch"""
    # Try both local and remote branch references
    for branch_ref in [f"origin/{branch}", branch]:
        try:
            result = subprocess.run(
                [
                    "git",
                    "log",
                    branch_ref,
                    f"--max-count={limit}",
                    "--pretty=format:%h (%s)",
                ],
                cwd=REPO_PATH,
                capture_output=True,
                text=True,
                check=True,
            )
            commits = result.stdout.strip().split("\n") if result.stdout.strip() else []
            return [{"hash": commit, "branch": branch} for commit in commits]
        except subprocess.CalledProcessError as e:
            print(
                f"Failed to get commits from {branch_ref}: {e.stderr if hasattr(e, 'stderr') else str(e)}"
            )
            continue
    return []


@app.get("/api/commits")
async def get_commits():
    """API endpoint to fetch git commits from main and integration branches"""
    main_commits = get_git_commits("main", limit=5)
    integration_commits = get_git_commits("integration", limit=5)

    return {"commits": main_commits + integration_commits}


@app.post("/api/launch")
async def launch(commit: str = Form(...), params: str = Form("")):
    """Execute the bash script with selected commit and parameters"""
    try:
        commit_hash = commit.split()[0]
        script_path = os.path.join(APP_PATH, "launch.sh")

        command = [script_path, commit_hash] + params.split()

        result = subprocess.run(
            command, cwd=REPO_PATH, capture_output=True, text=True, timeout=300
        )

        output = result.stdout + result.stderr

        if result.returncode == 0:
            return {
                "success": True,
                "output": output or "Script executed successfully!",
            }
        else:
            return {
                "success": False,
                "error": f"Script failed with exit code {result.returncode}\n\n{output}",
            }

    except FileNotFoundError:
        return {
            "success": False,
            "error": f"Script not found: {script_path}\n\nPlease create your launch.sh script",
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "Script execution timed out (>5 minutes)"}
    except Exception as e:
        return {"success": False, "error": f"Unexpected error: {str(e)}"}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
