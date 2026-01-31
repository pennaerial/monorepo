from fastapi import FastAPI, Form
from fastapi.middleware.cors import CORSMiddleware
import subprocess
import os
import time

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
async def launch(commit: str = Form(...), params: str = Form(""), target: str = Form(""), password: str = Form("")):
    """Deploy build and launch UAV with specified parameters"""
    try:
        if not target:
            return {"success": False, "error": "Target (pi@hostname) is required"}
        
        # Extract short commit hash (first 7 characters)
        commit_hash = commit.split()[0]
        short_sha = commit_hash[:7]
        
        # Parse params string to extract values
        # Format: "mission_name:=value uav_debug:=value ..."
        param_dict = {}
        if params:
            for item in params.split():
                if ":=" in item:
                    key, value = item.split(":=", 1)
                    param_dict[key] = value
        
        # Set up environment with password if provided
        env = os.environ.copy()
        if password:
            env["SSHPASS"] = password
        
        # Step 1: Deploy the build using push-deploy.sh
        script_path = os.path.join(REPO_PATH, "controls/sae_2025_ws/scripts/push-deploy.sh")
        deploy_command = [script_path, "--build", short_sha, target]
        
        deploy_result = subprocess.run(
            deploy_command,
            cwd=REPO_PATH,
            env=env,
            capture_output=True,
            text=True,
            timeout=600  # 10 minutes for deployment
        )
        
        deploy_output = deploy_result.stdout + deploy_result.stderr
        
        if deploy_result.returncode != 0:
            debug_info = f"\n\n{'='*50}\nDEBUG (commands that were run):\n{'='*50}\nDeploy: {' '.join(deploy_command)}\n\nParsed params:\n{chr(10).join(f'{k} = {v}' for k, v in param_dict.items())}"
            return {"success": False, "error": f"Deployment failed with exit code {deploy_result.returncode}\n\n{deploy_output}{debug_info}"}
        
        # Step 2: SSH into Pi and run the launch command with environment variables
        # Build environment variable exports for all parameters
        env_exports = []
        if param_dict.get('mission_name'):
            env_exports.append(f"export UAV_MISSION={param_dict['mission_name']}")
        if 'sim' in param_dict:
            env_exports.append(f"export UAV_SIM={param_dict['sim'].lower()}")
        if 'uav_debug' in param_dict:
            env_exports.append(f"export UAV_DEBUG={param_dict['uav_debug'].lower()}")
        if 'vision_debug' in param_dict:
            env_exports.append(f"export UAV_VISION_DEBUG={param_dict['vision_debug'].lower()}")
        if 'run_mission' in param_dict:
            env_exports.append(f"export UAV_RUN_MISSION={param_dict['run_mission'].lower()}")
        if 'vehicle_type' in param_dict:
            env_exports.append(f"export UAV_VEHICLE_TYPE={param_dict['vehicle_type']}")
        if 'save_vision' in param_dict:
            env_exports.append(f"export UAV_SAVE_VISION={param_dict['save_vision'].lower()}")
        if 'servo_only' in param_dict:
            env_exports.append(f"export UAV_SERVO_ONLY={param_dict['servo_only'].lower()}")
        if 'camera_offsets' in param_dict:
            # Camera offsets are already in clean format: "x,y,z"
            offsets = param_dict['camera_offsets'].strip()
            env_exports.append(f"export UAV_CAMERA_OFFSETS={offsets}")
        
        env_string = " && ".join(env_exports) if env_exports else ""
        
        # SSH command to run the launch
        ssh_base = ["sshpass", "-e"] if password else []
        
        # Build the launch command with environment variables
        # IMPORTANT: Export env vars AFTER sourcing activate.sh to ensure they override any defaults
        if env_string:
            launch_cmd = f"cd ~/uav_ws && source activate.sh && {env_string} && ros2 launch uav main.launch.py"
        else:
            launch_cmd = f"cd ~/uav_ws && source activate.sh && ros2 launch uav main.launch.py"
        
        ssh_command = ssh_base + ["ssh", "-o", "StrictHostKeyChecking=no", target, launch_cmd]
        
        # Run SSH command in background (don't wait for it to complete)
        launch_result = subprocess.Popen(
            ssh_command,
            cwd=REPO_PATH,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Wait briefly to catch any immediate errors
        time.sleep(2)
        
        if launch_result.poll() is not None:
            # Process already exited, likely an error
            stdout, stderr = launch_result.communicate()
            debug_info = f"\n\n{'='*50}\nDEBUG (commands that were run):\n{'='*50}\nDeploy: {' '.join(deploy_command)}\n\nSSH: {' '.join(ssh_command)}\n\nLaunch (on Pi): {launch_cmd}\n\nEnv vars: {env_string or '(none)'}\n\nParsed params:\n{chr(10).join(f'{k} = {v}' for k, v in param_dict.items())}"
            return {"success": False, "error": f"Launch failed:\n{stdout}\n{stderr}{debug_info}"}
        
        # Success: show deployment output + debug info for easier troubleshooting
        debug_info = f"\n\n{'='*50}\nDEBUG (commands run, for troubleshooting):\n{'='*50}\nDeploy: {' '.join(deploy_command)}\n\nSSH: {' '.join(ssh_command)}\n\nLaunch (on Pi): {launch_cmd}\n\nEnv vars: {env_string or '(none)'}\n\nParsed params:\n{chr(10).join(f'{k} = {v}' for k, v in param_dict.items())}"
        full_output = deploy_output + f"\n\nUAV is starting on {target}..." + debug_info
        
        return {"success": True, "output": full_output}

    except FileNotFoundError:
        return {"success": False, "error": f"Script not found: {script_path}\n\nMake sure push-deploy.sh exists"}
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "Deployment timed out (>10 minutes)"}
    except Exception as e:
        return {"success": False, "error": f"Unexpected error: {str(e)}"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
