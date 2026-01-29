#!/bin/bash
# push-deploy.sh - Deploy ROS 2 build to Raspberry Pi from your laptop
#
# Usage:
#   ./push-deploy.sh pi@192.168.1.50              # Deploy latest to one Pi
#   ./push-deploy.sh pi@drone1 pi@drone2          # Deploy to multiple Pis
#   ./push-deploy.sh --build abc123f pi@drone     # Deploy specific build
#   ./push-deploy.sh --local ./build.tar.gz pi@drone  # Deploy local file
#   ./push-deploy.sh --list                       # List available builds
#
# Environment:
#   GITHUB_REPO     - Repository (default: auto-detect from git remote)
#   REMOTE_DIR      - Install directory on Pi (default: ~/uav_ws)
#   SSH_KEY         - Path to SSH key (optional)
#

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }
step() { echo -e "${BLUE}[STEP]${NC} $1"; }

# Configuration
REMOTE_DIR="${REMOTE_DIR:-~/uav_ws}"
BUILD_SHA=""
LOCAL_FILE=""
TARGETS=()

# Auto-detect GitHub repo from git remote
detect_repo() {
    if [ -n "$GITHUB_REPO" ]; then
        echo "$GITHUB_REPO"
        return
    fi
    
    local remote_url
    remote_url=$(git remote get-url origin 2>/dev/null || echo "")
    
    if [[ "$remote_url" =~ github\.com[:/]([^/]+/[^/.]+) ]]; then
        echo "${BASH_REMATCH[1]}"
    else
        error "Could not detect GITHUB_REPO. Set it manually: export GITHUB_REPO=org/repo"
    fi
}

# SSH options
ssh_opts() {
    local opts="-o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"
    if [ -n "$SSH_KEY" ]; then
        opts="$opts -i $SSH_KEY"
    fi
    echo "$opts"
}

# List available builds
list_builds() {
    local repo
    repo=$(detect_repo)
    info "Fetching builds from $repo..."
    curl -s "https://api.github.com/repos/$repo/releases" | \
        jq -r '.[] | select(.tag_name | startswith("build-")) | "\(.tag_name | ltrimstr("build-"))\t\(.published_at)\t\(.name)"' | \
        head -20 | \
        column -t -s $'\t'
}

# Get download URL for a build
get_build_url() {
    local repo sha
    repo=$(detect_repo)
    sha="${1:-}"
    
    if [ -z "$sha" ]; then
        # Get latest
        curl -s "https://api.github.com/repos/$repo/releases" | \
            jq -r '[.[] | select(.tag_name | startswith("build-"))][0].assets[0].browser_download_url'
    else
        # Get specific build
        curl -s "https://api.github.com/repos/$repo/releases/tags/build-$sha" | \
            jq -r '.assets[0].browser_download_url'
    fi
}

# Download build artifact
download_build() {
    local url="$1"
    local dest="$2"
    
    if [ "$url" == "null" ] || [ -z "$url" ]; then
        error "No build found. Run with --list to see available builds."
    fi
    
    info "Downloading: $url"
    curl -L -o "$dest" "$url"
}

# Deploy to a single Pi
deploy_to_pi() {
    local target="$1"
    local tarball="$2"
    local filename
    filename=$(basename "$tarball")
    
    step "Deploying to $target"
    
    # Check connectivity
    info "Checking connection..."
    if ! ssh $(ssh_opts) "$target" "echo 'Connected'" 2>/dev/null; then
        error "Cannot connect to $target"
    fi
    
    # Create remote directory
    info "Preparing remote directory..."
    ssh $(ssh_opts) "$target" "mkdir -p $REMOTE_DIR"
    
    # Copy tarball
    info "Copying build artifact..."
    scp $(ssh_opts) "$tarball" "$target:$REMOTE_DIR/$filename"
    
    # Extract and set up on remote
    info "Extracting and configuring..."
    ssh $(ssh_opts) "$target" bash << REMOTE_SCRIPT
set -e
cd $REMOTE_DIR

# Backup existing install
if [ -d "install" ]; then
    echo "Backing up existing install..."
    rm -rf install.bak src.bak
    mv install install.bak 2>/dev/null || true
    mv src src.bak 2>/dev/null || true
fi

# Extract
echo "Extracting $filename..."
tar -xzf "$filename"
rm "$filename"

# Create activation script
cat > activate.sh << 'EOF'
#!/bin/bash
SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
cd "\$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash
export UAV_SIM=false
echo "ROS 2 workspace activated (sim=false)"
EOF
chmod +x activate.sh

# Create run script  
cat > run.sh << 'EOF'
#!/bin/bash
SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
cd "\$SCRIPT_DIR"
source activate.sh
ros2 launch uav main.launch.py "\$@"
EOF
chmod +x run.sh

echo "Deployment complete!"
cat install/BUILD_INFO.txt 2>/dev/null || true
REMOTE_SCRIPT

    info "✓ Deployed to $target"
}

# Parse arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --list|-l)
                list_builds
                exit 0
                ;;
            --build|-b)
                BUILD_SHA="$2"
                shift 2
                ;;
            --local|-f)
                LOCAL_FILE="$2"
                shift 2
                ;;
            --help|-h)
                echo "Usage: $0 [OPTIONS] TARGET [TARGET...]"
                echo ""
                echo "Deploy ROS 2 builds to Raspberry Pi(s) via SSH"
                echo ""
                echo "Options:"
                echo "  --list, -l              List available builds"
                echo "  --build, -b SHA         Deploy specific build by short SHA"
                echo "  --local, -f FILE        Deploy a local .tar.gz file"
                echo "  --help, -h              Show this help"
                echo ""
                echo "Examples:"
                echo "  $0 pi@192.168.1.50                    # Latest build to one Pi"
                echo "  $0 pi@drone1 pi@drone2                # Latest to multiple Pis"  
                echo "  $0 --build abc123f pi@drone           # Specific build"
                echo "  $0 --local ./my-build.tar.gz pi@drone # Local file"
                echo ""
                echo "Environment:"
                echo "  GITHUB_REPO   Repository (auto-detected from git)"
                echo "  REMOTE_DIR    Install path on Pi (default: ~/uav_ws)"
                echo "  SSH_KEY       Path to SSH key file"
                exit 0
                ;;
            -*)
                error "Unknown option: $1"
                ;;
            *)
                TARGETS+=("$1")
                shift
                ;;
        esac
    done
    
    if [ ${#TARGETS[@]} -eq 0 ]; then
        error "No target specified. Usage: $0 [OPTIONS] pi@hostname"
    fi
}

# Main
main() {
    parse_args "$@"
    
    local tarball
    local cleanup=false
    
    # Get the build artifact
    if [ -n "$LOCAL_FILE" ]; then
        if [ ! -f "$LOCAL_FILE" ]; then
            error "Local file not found: $LOCAL_FILE"
        fi
        tarball="$LOCAL_FILE"
        info "Using local file: $tarball"
    else
        # Download from GitHub
        local url
        url=$(get_build_url "$BUILD_SHA")
        
        tarball="/tmp/ros2-build-$$.tar.gz"
        cleanup=true
        
        download_build "$url" "$tarball"
    fi
    
    # Deploy to each target
    local failed=()
    for target in "${TARGETS[@]}"; do
        if deploy_to_pi "$target" "$tarball"; then
            :
        else
            failed+=("$target")
        fi
    done
    
    # Cleanup
    if [ "$cleanup" = true ]; then
        rm -f "$tarball"
    fi
    
    # Summary
    echo ""
    info "===== Deployment Summary ====="
    info "Successful: $((${#TARGETS[@]} - ${#failed[@]}))/${#TARGETS[@]}"
    
    if [ ${#failed[@]} -gt 0 ]; then
        warn "Failed: ${failed[*]}"
        exit 1
    fi
    
    echo ""
    info "To run on Pi:"
    echo "  ssh ${TARGETS[0]}"
    echo "  cd $REMOTE_DIR"
    echo "  ./run.sh"
}

main "$@"

