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

@app.get("/api/commits")
async def get_commits():
    """API endpoint to fetch recent commits from all branches"""
    try:
        # Get 50 most recent commits from all branches with branch decoration
        result = subprocess.run(
            ["git", "log", "--all", "--max-count=50", "--pretty=format:%h (%s)|%D"],
            cwd=REPO_PATH,
            capture_output=True,
            text=True,
            check=True
        )
        
        commits = []
        for line in result.stdout.strip().split("\n"):
            if not line:
                continue
            
            parts = line.split("|")
            commit_hash = parts[0]
            refs = parts[1] if len(parts) > 1 else ""
            
            # Extract branch name from refs
            branch = "unknown"
            if refs:
                for ref in refs.split(","):
                    ref = ref.strip()
                    if ref.startswith("origin/") and "HEAD" not in ref:
                        branch = ref.replace("origin/", "")
                        break
            
            commits.append({"hash": commit_hash, "branch": branch})
        
        return {"commits": commits}
    except subprocess.CalledProcessError:
        return {"commits": []}

@app.post("/api/launch")
async def launch(commit: str = Form(...), params: str = Form(""), target: str = Form("")):
    """Execute push-deploy.sh with selected commit hash"""
    try:
        # Extract short commit hash (first 7 characters)
        commit_hash = commit.split()[0]
        short_sha = commit_hash[:7]
        
        # Path to push-deploy.sh
        script_path = os.path.join(REPO_PATH, "controls/sae_2025_ws/scripts/push-deploy.sh")

        # Build command: push-deploy.sh --build <sha> <target>
        command = [script_path, "--build", short_sha]
        
        # Add target if provided (e.g., "pi@192.168.1.50")
        if target:
            command.append(target)

        result = subprocess.run(
            command,
            cwd=REPO_PATH,
            capture_output=True,
            text=True,
            timeout=600  # 10 minutes for deployment
        )

        output = result.stdout + result.stderr

        if result.returncode == 0:
            return {"success": True, "output": output or "Deployment completed successfully!"}
        else:
            return {"success": False, "error": f"Deployment failed with exit code {result.returncode}\n\n{output}"}

    except FileNotFoundError:
        return {"success": False, "error": f"Script not found: {script_path}\n\nMake sure push-deploy.sh exists"}
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "Deployment timed out (>10 minutes)"}
    except Exception as e:
        return {"success": False, "error": f"Unexpected error: {str(e)}"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
