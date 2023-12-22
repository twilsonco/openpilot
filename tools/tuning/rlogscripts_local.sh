#!/usr/bin/bash

DONGLEID="$1"

STARTDIR="$2"

OUTDIR="$STARTDIR"/../rlogsout

mkdir -p "$OUTDIR"

cd "$STARTDIR"

echo "Starting"

for i in *; do
  cd "$STARTDIR"
  if [ -f "$i"/rlog ]; then
    cp "$i"/rlog "$OUTDIR/$DONGLEID""_$i"--rlog
    bzip2 -f --verbose "$OUTDIR/$DONGLEID""_$i"--rlog # >> /data/getrlogs.txt 2>&1
    # cd /data/openpilot
    # ./tools/tuning/lat.py --path "/data/rlogs"
  else
    cp --verbose "$i"/rlog.bz2 "$OUTDIR/$DONGLEID""_$i"--rlog.bz2 # >> /data/getrlogs.txt 2>&1
    # cd /data/openpilot
    # ./tools/tuning/lat.py --path "/data/rlogs"
  fi
done
