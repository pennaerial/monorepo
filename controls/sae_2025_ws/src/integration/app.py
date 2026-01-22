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
            check=True
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return os.getcwd()

REPO_PATH = get_git_root()
APP_PATH = os.path.join(REPO_PATH, "controls/sae_2025_ws/src/integration")

def get_git_commits(limit=10):
    """Fetch recent git commits with hash and message"""
    try:
        result = subprocess.run(
            ["git", "log", f"--max-count={limit}", "--pretty=format:%h (%s)"],
            cwd=REPO_PATH,
            capture_output=True,
            text=True,
            check=True
        )
        commits = result.stdout.strip().split("\n")
        return commits
    except subprocess.CalledProcessError:
        return []

@app.get("/api/commits")
async def get_commits():
    """API endpoint to fetch git commits"""
    commits = get_git_commits()
    return {"commits": commits}

@app.post("/api/launch")
async def launch(commit: str = Form(...), params: str = Form("")):
    """Execute the bash script with selected commit and parameters"""
    try:
        commit_hash = commit.split()[0]
        script_path = os.path.join(APP_PATH, "launch.sh")

        command = [script_path, commit_hash] + params.split()

        result = subprocess.run(
            command,
            cwd=REPO_PATH,
            capture_output=True,
            text=True,
            timeout=300
        )

        output = result.stdout + result.stderr

        if result.returncode == 0:
            return {"success": True, "output": output or "Script executed successfully!"}
        else:
            return {"success": False, "error": f"Script failed with exit code {result.returncode}\n\n{output}"}

    except FileNotFoundError:
        return {"success": False, "error": f"Script not found: {script_path}\n\nPlease create your launch.sh script"}
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "Script execution timed out (>5 minutes)"}
    except Exception as e:
        return {"success": False, "error": f"Unexpected error: {str(e)}"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
