#!/usr/bin/env/bash

rlogdir="/Volumes/video/scratch-video/rlogs"
latdir="/Volumes/video/scratch-video/latfiles"

cd "$rlogdir"

for make in *; do
  makedir="$rlogdir/$make"
  if [ ! -d "$makedir" ]; then
    continue
  fi
  cd "$makedir"
  for car in *; do
    curdir="$makedir/$car"
    if [ ! -d "$curdir" ]; then
      continue
    fi
    cd "$curdir"
    outdir="$latdir/$make/$car"
    mkdir -p "$outdir"
    for dongle in *; do
      if [ ! -d "$curdir/$dongle" ]; then
        continue
      fi
      /Users/haiiro/NoSync/opfeedforward/openpilot/tools/tuning/lat.py --preprocess --path "$curdir/$dongle" --outpath "$outdir"
    done
  done
done

echo "Done"