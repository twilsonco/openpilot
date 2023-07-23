#!/opt/homebrew/bin/bash

# Set the folder with images 
IMG_FOLDER="/Users/haiiro/NoSync/gif_grid"
mkdir -p "$IMG_FOLDER"
cd "$IMG_FOLDER"
img_count=0

RES=100
PAD=$((RES/10))
DELAY_START=200
DELAY_END=3
GRID_SIZES=(3 5 7)
DELAY_STEPS=${#GRID_SIZES[@]}
ANIMATION_SECONDS=8
SECONDS_PER_GRID=1
OUT_FRAMERATE=33
TOTAL_FRAMES=$((ANIMATION_SECONDS*OUT_FRAMERATE))
FRAMES_PER_STEP=$((TOTAL_FRAMES/DELAY_STEPS))

SECONDS_PER_STEP=$((ANIMATION_SECONDS/DELAY_STEPS))
total_grid_num=0

# Create a temp folder for the image grids
mkdir -p /tmp/image_grids
rm -rf /tmp/image_grids/*

# Get a list of all jpg images in the folder
IMAGES=("$IMG_FOLDER"/*)
IMG_COUNT=${#IMAGES[@]}
echo "Found $IMG_COUNT images"

DELAY_STEP=0
DELAY=$DELAY_START
shuffled_images=( $(shuf -e "${IMAGES[@]}") )

for g in ${GRID_SIZES[@]}; do
  g2=$((g*g))
  
  res=$((RES*g/(g-2)))
  pad=$((PAD*g/(g-2)))

  tilesize=$((res-pad/2))
  tilepad=$((res-tilesize))

  # Compute max_frames using delay (in 1/100ths of a second per frame) 
  # and the nubmer of seconds per delay step
  FRAMERATE=$((100/DELAY+1))
  if [[ $OUT_FRAMERATE -lt $FRAMERATE ]]; then
    FRAMERATE=$OUT_FRAMERATE
    DUP_FRAMES=0
  else
    # Compute number of duplicate frames necessary to achieve the OUT_FRAMERATE
    DUP_FRAMES=$((OUT_FRAMERATE/FRAMERATE)) 
  fi
  DELAY_STEP=$((DELAY_STEP+1))
  DELAY=$(((DELAY+DELAY_END)/2))
  if [[ $DELAY_STEP -eq $((DELAY_STEPS-1)) ]]; then
    DELAY=$DELAY_END
  fi

  finished=false
  count=0
  grid_images=""
  grid_count=0
  while [[ $grid_count -lt $FRAMES_PER_STEP ]]; do
    shuffled_images=( $(shuf -e "${IMAGES[@]}") )
    for i in "${shuffled_images[@]}"; do
      if [[ $count -lt $g2 ]]; then
        grid_images+="$i "
        count=$((count+1))
        img_count=$((img_count+1))
      else
        # Composite the grid
        # pad image number with up to 4 zeros
        image_num=$total_grid_num
        image_num=$(printf "%06d" $image_num)
        img=/tmp/image_grids/grid_$image_num.jpg
        img_check=/tmp/image_grids/grid_$image_num-0.jpg
        img_glob=/tmp/image_grids/grid_$image_num*.jpg

        img_orig=/tmp/image_grids/grid_tmp_$image_num-orig.jpg
        img_orig_check=/tmp/image_grids/grid_tmp_$image_num-orig-0.jpg
        img_orig_glob=/tmp/image_grids/grid_tmp_$image_num-orig*.jpg
        montage ${grid_images[@]} -tile "$g"x"$g" -geometry "$tilesize"x"$tilesize"+"$tilepad"+"$tilepad" $img_orig
        grid_images=""
        count=0
        if [[ -f $img_orig_check ]]; then
          rm -f $img_orig_glob
          echo "Bad output from montage, redoing..."
          continue
        fi   
        # Crop and resize the image
        echo "Cropping and resizing $img"

        if [[ $grid_count -eq 0 ]]; then
          img_w=$(identify -format "%w" $img_orig)
          img_h=$(identify -format "%h" $img_orig)
          echo "Image size: $img_w x $img_h"
          crop_len=$((img_w*(g-2)/g))
          crop_pos=$((img_w/2-crop_len/2))

          # Need to smoothly zoom out of the image with each frame.
          # All the grids are odd numbers, so the zoom is such that
          # when grid_count == 0, the outermost row and columm of images
          # are cropped out. When grid_count == FRAMES_PER_STEP, the
          # entire image is visible.
          # This is done by cropping and then resizing the image.
          # The crop and zoom amount depend on grid_count, FRAMES_PER_STEP,
          # and the grid size.
          len_step=$(((img_w-crop_len)/FRAMES_PER_STEP))
          echo "Crop len: $crop_len, crop pos: $crop_pos, len step: $len_step"
        fi

        if [[ $((crop_len+len_step)) -le $img_w ]]; then
          convert $img_orig -crop "$crop_len"x"$crop_len"+"$crop_pos"+"$crop_pos" -resize "$RES"x"$RES" $img
          if [[ -f $img_check ]]; then
          exit 1
            rm -f $img_glob
            rm -f $img_orig_glob
            echo "Bad output from convert, redoing..."
            continue
          fi
          echo "Crop: $crop_len x $crop_len + $crop_pos + $crop_pos"
          crop_len=$((crop_len+len_step))
          crop_pos=$((img_w/2-crop_len/2))
        else
          cp $img_orig $img
        fi
        echo "`date` Generating grid $total_grid_num of $TOTAL_FRAMES"
        total_grid_num=$((total_grid_num+1))
        grid_count=$((grid_count+1))
        for (( j=0; j<$DUP_FRAMES; j++ )); do
          # echo "Duplicating frame $total_grid_num"
          total_grid_num=$((total_grid_num+1))
          grid_count=$((grid_count+1))
          # pad image number with up to 4 zeros
          image_num1=$total_grid_num
          image_num1=$(printf "%06d" $image_num1)
          img1=/tmp/image_grids/grid_$image_num1.jpg
          if [[ $((crop_len+len_step)) -le $img_w ]]; then
            convert $img_orig -crop "$crop_len"x"$crop_len"+"$crop_pos"+"$crop_pos" -resize "$RES"x"$RES" $img1
            echo "Crop: $crop_len x $crop_len + $crop_pos + $crop_pos"
            crop_len=$((crop_len+len_step))
            crop_pos=$((img_w/2-crop_len/2))
          else
            cp $img $img1
          fi
          if [[ $grid_count -ge $FRAMES_PER_STEP ]]; then
            break
          fi
        done
        rm $img_orig
        if [[ $grid_count -ge $FRAMES_PER_STEP ]]; then
          break
        fi
      fi
    done
  done
done
# Combine all grids into an animated gif
echo "Generating gif"
convert -delay $DELAY_END -loop 3 /tmp/image_grids/*.jpg animated.gif

echo "Animated gif created at $(pwd)/animated.gif"

# rm -rf /tmp/image_grids*
