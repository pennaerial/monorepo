#!/usr/bin/env bash

export SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ci_conf="${SCRIPT_DIR}/ci.conf"

if [[ -f "${ci_conf}" ]]; then
    source "${ci_conf}" # imports all defined constants from ci.conf
else
    echo "Error: Configuration file not found at ${ci_conf}" >&2
    exit 1
fi


# Print out submodules
echo "Detected submodules: ${#MANAGED_SUBMODULES[@]}"
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
    )
done

echo "All managed submodules updated to latest on their default branch."

# Commit the bumped submodule pins in the superproject
git config user.name "github-actions[bot]"
git config user.email "github-actions[bot]@users.noreply.github.com"

git add "${MANAGED_SUBMODULES[@]}"

if git diff --cached --quiet; then
    echo "No submodule changes to commit."
else
    git commit -m "NIGHTLY: bump submodule to most up-to-date"
    git push origin HEAD
    echo "Committed and pushed submodule updates."
fi
