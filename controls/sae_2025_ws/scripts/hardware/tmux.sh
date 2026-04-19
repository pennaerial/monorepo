#!/usr/bin/env bash
# Attach to or create a named tmux session.
# Standard sessions: payload-mission, payload-rqt, payload-shell
#
# Usage:
#   tmux.sh <name> [command...]
#   tmux.sh payload-mission ./runtime_fleet.sh
#   tmux.sh payload-rqt rqt
#   tmux.sh payload-shell
#   tmux.sh          # list active sessions
#
# Detach without killing: Ctrl-B D

set -euo pipefail

if [[ $# -eq 0 ]]; then
    tmux ls 2>/dev/null || printf '(no active sessions)\n'
    exit 0
fi

SESSION="$1"
shift

if tmux has-session -t "$SESSION" 2>/dev/null; then
    exec tmux attach-session -t "$SESSION"
fi

if [[ $# -gt 0 ]]; then
    tmux new-session -d -s "$SESSION" -x 220 -y 50
    tmux send-keys -t "$SESSION" "$*" Enter
    exec tmux attach-session -t "$SESSION"
else
    exec tmux new-session -s "$SESSION" -x 220 -y 50
fi
