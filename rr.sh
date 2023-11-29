#!/usr/bin/sh
# rr - rebootless restart
## old version
# tmux kill-session -t comma; rm -f /tmp/safe_staging_overlay.lock; tmux new -s comma -d "/data/openpilot/launch_openpilot.sh"
sudo systemctl restart comma
echo "Restarting openpilot... Exit code $?"
