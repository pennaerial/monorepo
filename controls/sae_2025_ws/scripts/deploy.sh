#!/bin/bash
# deploy.sh - Download and deploy the latest ROS 2 build to Raspberry Pi
#
# Usage:
#   ./deploy.sh              # Get latest build
#   ./deploy.sh abc123f      # Get specific build by short SHA
#   ./deploy.sh --list       # List available builds
#
# Requirements:
#   - curl, jq, tar
#   - ROS 2 Humble installed on the Pi
#

set -e

# Configuration - Update these for your repo
GITHUB_REPO="${GITHUB_REPO:-your-org/your-repo}"  # Override with env var or edit here
INSTALL_DIR="${INSTALL_DIR:-$HOME/ros2_ws}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# Check dependencies
check_deps() {
    for cmd in curl jq tar; do
        if ! command -v $cmd &> /dev/null; then
            error "$cmd is required but not installed. Install with: sudo apt install $cmd"
        fi
    done
}

# List available builds
list_builds() {
    info "Fetching available builds from $GITHUB_REPO..."
    curl -s "https://api.github.com/repos/$GITHUB_REPO/releases" | \
        jq -r '.[] | select(.tag_name | startswith("build-")) | "\(.tag_name)\t\(.published_at)\t\(.name)"' | \
        head -20 | \
        column -t -s $'\t'
}

# Get latest release info
get_latest() {
    curl -s "https://api.github.com/repos/$GITHUB_REPO/releases" | \
        jq -r '[.[] | select(.tag_name | startswith("build-"))][0]'
}

# Get release by tag
get_release_by_tag() {
    local tag="build-$1"
    curl -s "https://api.github.com/repos/$GITHUB_REPO/releases/tags/$tag"
}

# Download and extract build
deploy_build() {
    local release_json="$1"
    local tag=$(echo "$release_json" | jq -r '.tag_name')
    local download_url=$(echo "$release_json" | jq -r '.assets[0].browser_download_url')
    local asset_name=$(echo "$release_json" | jq -r '.assets[0].name')
    
    if [ "$download_url" == "null" ] || [ -z "$download_url" ]; then
        error "No download URL found for release"
    fi
    
    info "Deploying $tag"
    info "Download URL: $download_url"
    
    # Create install directory
    mkdir -p "$INSTALL_DIR"
    cd "$INSTALL_DIR"
    
    # Download
    info "Downloading $asset_name..."
    curl -L -o "$asset_name" "$download_url"
    
    # Backup existing install if present
    if [ -d "install" ]; then
        warn "Backing up existing workspace to backup/"
        rm -rf backup
        mkdir -p backup
        mv install src launch_params.yaml backup/ 2>/dev/null || true
    fi
    
    # Extract
    info "Extracting..."
    tar -xzvf "$asset_name"
    
    # Cleanup
    rm "$asset_name"
    
    # Create activation script
    cat > activate.sh << 'ACTIVATE_EOF'
#!/bin/bash
# Source this file to activate the ROS 2 workspace
# Usage: source activate.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash

# Ensure hardware mode (sim=false) is used
export UAV_SIM=false

echo "ROS 2 workspace activated!"
echo ""
echo "Build info:"
cat install/BUILD_INFO.txt 2>/dev/null || echo "No build info available"
echo ""
echo "To run: ros2 launch uav main.launch.py"
echo "Config: edit launch_params.yaml or use UAV_MISSION=name"
ACTIVATE_EOF
    chmod +x activate.sh
    
    # Create a run script for convenience
    cat > run.sh << 'RUN_EOF'
#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source activate.sh
ros2 launch uav main.launch.py "$@"
RUN_EOF
    chmod +x run.sh
    
    info "Deployment complete!"
    echo ""
    info "To use the workspace:"
    echo "  cd $INSTALL_DIR"
    echo "  source activate.sh"
    echo ""
    info "Build info:"
    cat install/BUILD_INFO.txt 2>/dev/null || warn "No build info found"
}

# Main
main() {
    check_deps
    
    case "${1:-}" in
        --list|-l)
            list_builds
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS] [SHORT_SHA]"
            echo ""
            echo "Options:"
            echo "  --list, -l    List available builds"
            echo "  --help, -h    Show this help"
            echo ""
            echo "Examples:"
            echo "  $0              Deploy latest build"
            echo "  $0 abc123f      Deploy specific build"
            echo "  $0 --list       List available builds"
            echo ""
            echo "Environment variables:"
            echo "  GITHUB_REPO    Repository (default: $GITHUB_REPO)"
            echo "  INSTALL_DIR    Install directory (default: $INSTALL_DIR)"
            ;;
        "")
            info "Fetching latest build..."
            release=$(get_latest)
            if [ "$(echo "$release" | jq -r '.tag_name')" == "null" ]; then
                error "No builds found. Check GITHUB_REPO is set correctly: $GITHUB_REPO"
            fi
            deploy_build "$release"
            ;;
        *)
            info "Fetching build $1..."
            release=$(get_release_by_tag "$1")
            if [ "$(echo "$release" | jq -r '.tag_name')" == "null" ]; then
                error "Build $1 not found. Use --list to see available builds."
            fi
            deploy_build "$release"
            ;;
    esac
}

main "$@"

