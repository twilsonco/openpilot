#!/bin/bash

# Directory with original images  
IMG_DIR="/Users/haiiro/NoSync/gif_grid"
OUT_DIR="/Users/haiiro/NoSync/gif_grid_wm"
mkdir -p "$OUT_DIR"

# Directory with watermark images
WATERMARK_DIR="/Users/haiiro/NoSync/make_logos"

# Iterate through original images
for img in $IMG_DIR/*
do
  # Get image size
  width=$(identify -format %w "$img")
  height=$(identify -format %h "$img")
  if [[ $width == "" ]] || [[ $height == "" ]]; then
    continue
  fi

  # Calculate watermark size
  wm_width=$(($width*3/4))
  wm_height=$(($height*3/4))

  # Get filename
  filename=$(basename "$img")

  # Extract watermark name from filename
  watermark="${filename%%_*}"

  # Path to watermark image
  watermark_img="$WATERMARK_DIR/$watermark.png"
  
  if [[ -f "$watermark_img" ]]; then
    # make temporary copy of watermark image and save path to it
    watermark_img_tmp=$(mktemp)
    cp "$watermark_img" "$watermark_img_tmp"

    # Resize watermark
    convert "$watermark_img" -resize ${wm_width}x${wm_height} "$watermark_img_tmp"

    # Get watermark size after resizing
    wm_width=$(identify -format %w "$watermark_img_tmp")
    wm_height=$(identify -format %h "$watermark_img_tmp")

    # Calculate center position
    x=$(($width/2 - $wm_width/2))
    y=$(($height*4/10 - $wm_height/2))

    # Create label from filename, which is the second and third underscore-delimited word of the filename
    # and nothing more. So First get the basename of the file without the extension, then
    # get the second and third underscore-delimited word of the filename and replace underscore with space.
    # Then, if applicable, truncate the first hyphen and everything after.
    label=$(basename "$img" | cut -d_ -f2,3 | sed 's/\..*//g' | sed 's/_/\n/g')

    # Position label in bottom quarter
    label_y=$(($height/10-50))

    # Path to save watermarked image
    output="$OUT_DIR/watermarked_$filename"

    # Add watermark with imagemagick
    composite -dissolve 30 -geometry +$x+$y "$watermark_img_tmp" "$img" "$output"
    input="$output"
  else
    echo "No watermark found for $img"
    input="$img"
  fi
  # Add label
  convert "$output" -undercolor none -gravity south \
    -pointsize 300 -stroke '#000C' -annotate +0+$label_y "$label" \
    "$output"
  # break
done
