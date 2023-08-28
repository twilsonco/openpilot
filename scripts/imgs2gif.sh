#!/opt/homebrew/bin/bash

# Find common prefix
find_prefix() {
  local first_img="$1"
  local img_list=("${@:2}")
  local filename="$1"
  PREFIX="${filename%_*}"
  PREFIX="${PREFIX%.*}"

  # Set initial prefix
  # PREFIX=$(basename "$first_img" .png)

  # Find common prefix
  for img in "${img_list[@]}"; do
    NAME=$(basename "$img" .png)

    while [[ $NAME != $PREFIX* ]] && [[ "${PREFIX: -1}" != "_" ]]; do
      PREFIX=${PREFIX%?}
    done
    if [[ "${PREFIX: -2}" == "_" ]] && [[ "${PREFIX: -1}" == "b" ]]; then
      break
    elif [[ "${PREFIX: -2}" != "'" ]] && [[ "${PREFIX: -1}" == "." ]]; then
      break
    fi
  done

  echo "${PREFIX%_}"
}

# Get unique prefixes
get_prefixes() {

  img_list=("$1"/*.png)
  # printf "%s\n" "${img_list[@]}"

  declare -A prefixes
  
  # Loop over images
  for img in "${img_list[@]}"; do

    # Check if starts with existing prefix
    prefix_match=0
    for p in "${!prefixes[@]}"; do
      if [[ $img == *"$p"*png ]]; then
        prefix_match=1
        break
      fi
    done

    # If no match, find new prefix
    if [ $prefix_match -eq 0 ]; then
      filename=$(basename "$img")
      prefix=$(find_prefix "$filename" "${img_list[@]}")

      prefixes["$prefix"]=0
    fi
  done
  
  echo "${!prefixes[@]}"
}

get_prefixes1() {
  img_list=("$1"/*.png)
  declare -A prefixes

  for img in "${img_list[@]}"; do
    filename=$(basename "$img")
    if [[ $filename == *_b* ]]; then
      prefix=${filename%%_b*}
    else
      prefix=${filename%%.png}
    fi
    prefixes["$prefix"]=0
  done
  
  echo "${!prefixes[@]}"
}

# Main script

# Check arguments
if [ $# -ne 2 ]; then
  echo "Usage: $0 <fps> <dir>"
  exit 1
fi

FRAMERATE=$1
IMG_FOLDER=$2

# Get list of PNGs
img_list=$(ls "$IMG_FOLDER"/*.png)
tmpvar=("$IMG_FOLDER/"*.png)
num_images=${#tmpvar[@]}
echo "Processing $num_images total images"

# Get unique prefixes
PREFIXES="$(get_prefixes1 "$IMG_FOLDER")"
num_prefixes=$(echo "$PREFIXES" | wc -w | xargs) # xargs trims the whitespace :)

echo "Processing $num_prefixes cars"
pi=1
# Loop over prefixes  
for PREFIX in $PREFIXES; do
  echo "Processing $pi of $num_prefixes | $PREFIX"
  pi=$((pi+1))

  # Make temp dir
  TMP_DIR=$(mktemp -d)

  # Get sorted images
  IMG_LIST=("$IMG_FOLDER/$PREFIX"*\'.png)
  IMG_LIST+=("$IMG_FOLDER/$PREFIX".png)

  # Copy and rename
  i=1 
  n=${#IMG_LIST[@]}
  n=$((n-1))
  if [[ $n -eq 1 ]]; then
    continue
  fi
  for img in "${IMG_LIST[@]}"; do
    label="$PREFIX "
    if [[ $i -eq $((n+1)) ]]; then
      label+="(all)"
    else
      label+="($i/$n)"
    fi
    
    convert "$img" -resize 900x900 -gravity south \
          -pointsize 30 -stroke '#000C' -strokewidth 4 -annotate 0 "$label" \
          -pointsize 30 -stroke  none   -fill white    -annotate 0 "$label" \
          "$TMP_DIR/$i.png"
    i=$((i+1))
  done
  
  # Generate gif
  ffmpeg -y -framerate $FRAMERATE -i "$TMP_DIR/%d.png" -filter_complex "[0:v] palettegen" palette.png
  ffmpeg -y -framerate $FRAMERATE -i "$TMP_DIR/%d.png" -i palette.png -lavfi "paletteuse" "$IMG_FOLDER/$PREFIX.gif"

  # Clean up
  rm -r "$TMP_DIR"
done