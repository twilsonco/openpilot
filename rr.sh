#!/usr/bin/sh
# rr - rebootless restart
tmux kill-session -t comma; rm -f /tmp/safe_staging_overlay.lock; tmux new -s comma -d "/data/openpilot/launch_openpilot.sh"
echo "Restarting openpilot... Exit code $?"
