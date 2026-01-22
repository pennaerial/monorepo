#!/bin/bash

COMMIT_HASH=$1
shift

ORIGINAL_BRANCH=$(git rev-parse --abbrev-ref HEAD)
git checkout $COMMIT_HASH