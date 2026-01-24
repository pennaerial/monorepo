#!/bin/bash

# Get commit hash from first argument
COMMIT_HASH="$1"
SHORT_SHA=$(echo "$COMMIT_HASH" | cut -c1-7)
echo "SHORT_SHA: $SHORT_SHA"

# ssh xxxxx
# cd /.......

# git pull origin main

gh run download --name "ros2-build-${SHORT_SHA}" --dir ./dist
