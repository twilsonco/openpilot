#!/usr/bin/bash

DONGLEID="$1"

mkdir -p /data/rlogs
rm -f /data/rlogs/*rlog.bz2

cd /data/media/0/realdata

echo "Starting"

cd /data/media/0/realdata
for i in *; do
  cd /data/media/0/realdata
  if [ -f "$i"/rlog ]; then
    cp "$i"/rlog /data/rlogs/"$DONGLEID""_$i"--rlog
    bzip2 -f --verbose /data/rlogs/"$DONGLEID""_$i"--rlog # >> /data/getrlogs.txt 2>&1
    cd /data/openpilot
    ./tools/tuning/lat.py --path "/data/rlogs"
  else
    cp --verbose "$i"/rlog.bz2 /data/rlogs/"$DONGLEID""_$i"--rlog.bz2 # >> /data/getrlogs.txt 2>&1
    cd /data/openpilot
    ./tools/tuning/lat.py --path "/data/rlogs"
  fi
done
