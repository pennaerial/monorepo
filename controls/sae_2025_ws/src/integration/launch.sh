#!/bin/bash

# Get commit hash from first argument
COMMIT_HASH="$1"
SHORT_SHA=$(echo "$COMMIT_HASH" | cut -c1-7)
echo "SHORT_SHA: $SHORT_SHA"

ssh user@remote-host << EOF
  cd /path/to/deploy
  gh run download --name "ros2-build-${SHORT_SHA}" --dir ./dist

  cd ./dist
  tar -xzf ros2-build-${SHORT_SHA}.tar.gz

  echo "Deployment complete!"
EOF