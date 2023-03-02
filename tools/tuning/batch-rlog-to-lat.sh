#!/opt/homebrew/bin/bash

rlogdir="/Volumes/video/scratch-video/rlog_api"
latdir="/Volumes/video/scratch-video/latfiles"

cd "$rlogdir"

for car in *; do
  curdir="$rlogdir/$car"
  if [ ! -d "$curdir" ]; then
    continue
  fi
  cd "$curdir"
  outdir="$latdir/$car"
  mkdir -p "$outdir"
  for dongle in *; do
    if [ ! -d "$curdir/$dongle" ]; then
      continue
    fi
    /Users/haiiro/NoSync/opfeedforward/openpilot/tools/tuning/lat.py --preprocess --path "$curdir/$dongle" --outpath "$outdir"
  done
done

echo "Done"