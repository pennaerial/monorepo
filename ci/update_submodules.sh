#!/usr/bin/env bash
set -euo pipefail
# this script makes sure all submodules are correctly up-to-date
# There are managed submodules, which are pennair-owned and directly tested to be stable with monorepo.
# These managed submodules should always stay up-to-date with their 'main' branches.
#
# Then there are 3rd party submodules directly linked to monorepo (like px4_msgs). We don't want these
# submodules to be always up-to-date with their main branches because they are not directly tested against monorepo
# Their commit tags should be manually tested and updated




# covers any 3rd party submodules that we only want on pinned tag, not on default branch
# managed submodules will get manually checked out to default later
git submodule sync --recursive
git submodule update --init --recursive

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ci.conf" # imports all defined constants from ci.conf


# Print out submodules
for submodule in "${MANAGED_SUBMODULES[@]}"; do
    echo "${submodule}"
done
# Update each managed submodule to the latest commit on origin's default branch
for submodule_path in "${MANAGED_SUBMODULES[@]}"; do
    if [[ ! -d "${submodule_path}" ]]; then
        echo "Skipping ${submodule_path}: directory not found" >&2
        continue
    fi
    echo "==> Updating ${submodule_path}"
    (
        cd "${submodule_path}"
        git fetch origin
        default_branch="$(git remote show origin | awk '/HEAD branch/ {print $NF}')"
        echo "    default branch: ${default_branch}"
        git checkout "${default_branch}"
        git pull origin "${default_branch}"

        # Handle this submodule's own nested submodules, if any
        if [[ -f ".gitmodules" ]]; then
            echo "    syncing nested submodules for ${submodule_path}"
            git submodule sync --recursive
            git submodule update --init --recursive
        fi
    )
done
echo "All managed submodules updated to latest on their default branch."
# Commit the bumped submodule pins in the superproject
git config user.name "github-actions[bot]"
git config user.email "github-actions[bot]@users.noreply.github.com"
git add "${MANAGED_SUBMODULES[@]}"
if git diff --cached --quiet; then
    echo "No submodule changes to commit."
fi
