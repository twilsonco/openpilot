#!/usr/bin/env bash

rlogdir="/mnt/video/scratch-video/rlogs"
latdir="/mnt/video/scratch-video/parquet_nworby"

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
    # mkdir -p "$outdir"
    for dongle in *; do
      if [ ! -d "$curdir/$dongle" ]; then
        continue
      fi
      python3 /home/haiiro/openpilot-batch/openpilot/tools/tuning/process_rlogs.py "$curdir/$dongle" "$latdir/$car.parquet"
    done
  done
done

echo "Done"