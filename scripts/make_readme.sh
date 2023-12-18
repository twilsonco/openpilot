#!/bin/sh

# Define preface and postface strings
preface="[![intro_vid](https://github.com/twilsonco/openpilot/blob/log-info/img/comma-steering-control-vid.gif?raw=true)](https://youtu.be/G_SNRJCGKHs?si=4lMWhB3kcHj6Fu9E)

# Improving controls with log data

Provide feedback on lateral performance here: https://forms.gle/dtGQrY3TSkyQo6nu7

Here's plots of the [comma-steering-control](https://github.com/commaai/comma-steering-control) dataset and the data I've collected so far from the community.
Community log collection is in a limited state; only cars not in the comma-steering-control dataset will be collected for the time being.
I'll update here periodically so log contributors can see which speeds/angles need to be filled out.

Head to the Comma, Community, or SunnyPilot Discord server #tuning (or tuning-nnff) channels if you want to contribute or learn more!"
lat_file_count=167465
lat_file_list="## Current counts of collected logs (list updated Nov 18, 2023)\n\n
\`\`\`
volkswagen                              9948 (18G)
  VOLKSWAGEN TIGUAN 2ND GEN             474 (894M)
  VOLKSWAGEN SHARAN 2ND GEN             59 (105M)
  VOLKSWAGEN POLO 6TH GEN               528 (953M)
  VOLKSWAGEN PASSAT NMS                 1470 (2.5G)
  VOLKSWAGEN PASSAT 8TH GEN             657 (1.4G)
  VOLKSWAGEN JETTA 7TH GEN              359 (781M)
  VOLKSWAGEN GOLF 7TH GEN               747 (1.4G)
  VOLKSWAGEN GOLF 6TH GEN               37 (51M)
  VOLKSWAGEN ATLAS 1ST GEN              1010 (1.9G)
  SKODA KODIAQ 1ST GEN                  3324 (5.9G)
  SKODA KAROQ 1ST GEN                   575 (949M)
  SEAT LEON 3RD GEN                     49 (90M)
  AUDI A3 3RD GEN                       659 (1.1G)
toyota                                  26115 (52G)
  TOYOTA SIENNA 2018                    53 (102M)
  TOYOTA RAV4 HYBRID 2022               11 (16M)
  TOYOTA RAV4 HYBRID 2019               4438 (8.8G)
  TOYOTA RAV4 2019                      1194 (2.6G)
TOYOTA PRIUS v 2017                     2342 (4.7G)
  TOYOTA PRIUS TSS2 2021                6390 (14G)
  TOYOTA PRIUS 2017                     2503 (5.1G)
  TOYOTA HIGHLANDER HYBRID 2020         13 (18M)
  TOYOTA COROLLA TSS2 2019              722 (1.4G)
  TOYOTA COROLLA HYBRID TSS2 2019       2588 (4.9G)
  TOYOTA COROLLA 2017                   3589 (7.0G)
  TOYOTA CAMRY HYBRID 2021              1113 (2.1G)
  LEXUS RX 2020                         102 (198M)
  LEXUS ES 2019                         1057 (2.2G)
subaru                                  3066 (6.0G)
  SUBARU OUTBACK 6TH GEN                465 (833M)
  SUBARU OUTBACK 2015 - 2017            78 (151M)
  SUBARU LEGACY 7TH GEN                 503 (1021M)
  SUBARU LEGACY 2015 - 2018             1237 (2.6G)
  SUBARU IMPREZA SPORT 2020             99 (247M)
  SUBARU FORESTER 2017 - 2018           684 (1.3G)
nissan                                  1297 (2.1G)
  NISSAN LEAF 2018                      1297 (2.1G)
mazda                                   4993 (8.6G)
  MAZDA CX-9 2021                       16 (25M)
  MAZDA CX-9                            1294 (2.2G)
  MAZDA CX-5 2022                       105 (178M)
  MAZDA CX-5                            1370 (2.8G)
  MAZDA 3 2019                          2106 (3.3G)
  MAZDA 3 2014                          66 (109M)
  MAZDA 3                               35 (62M)
hyundai                                 46888 (93G)
  KIA STINGER GT2 2018                  92 (171M)
  KIA SPORTAGE 5TH GEN                  3197 (6.4G)
  KIA NIRO HYBRID 2ND GEN               376 (815M)
  KIA NIRO HYBRID 2021                  2905 (6.0G)
  KIA NIRO HYBRID 2019                  78 (149M)
  KIA NIRO EV 2020                      2111 (4.3G)
  KIA K8 HYBRID 1ST GEN                 1785 (3.8G)
  KIA EV6 2022                          4688 (9.7G)
  KIA CEED INTRO ED 2019                10512 (21G)
  HYUNDAI TUCSON HYBRID 4TH GEN         7 (5.8M)
  HYUNDAI SONATA HYBRID 2021            254 (509M)
  HYUNDAI SONATA 2020                   2908 (5.9G)
  HYUNDAI SANTA FE HYBRID 2022          11899 (23G)
  HYUNDAI SANTA FE 2022                 1 (4.0K)
  HYUNDAI SANTA FE 2019                 617 (1.4G)
  HYUNDAI PALISADE 2020                 857 (1.8G)
  HYUNDAI KONA ELECTRIC 2022            125 (255M)
  HYUNDAI IONIQ PHEV 2020               1612 (3.2G)
  HYUNDAI IONIQ HYBRID 2017-2019        15 (21M)
  HYUNDAI IONIQ 5 2022                  1267 (2.7G)
  HYUNDAI GENESIS 2015-2016             116 (219M)
  GENESIS GV70 1ST GEN                  1307 (2.8G)
  GENESIS (DH)                          159 (328M)
honda                                   6178 (12G)
  HONDA RIDGELINE 2017                  3348 (6.4G)
  HONDA HR-V 2023                       129 (231M)
  HONDA HRV                             1 (4.0K)
  HONDA CLARITY 2018                    930 (1.8G)
  HONDA CIVIC (BOSCH) 2019              526 (1.1G)
  HONDA CIVIC 2022                      25 (44M)
  HONDA ACCORD 2018                     1218 (2.4G)
gm                                      49871 (91G)
  GMC YUKON NO ACC                      47 (72M)
  GMC ACADIA DENALI 2018                6633 (13G)
  CHEVROLET VOLT PREMIER 2018           11864 (22G)
  CHEVROLET VOLT PREMIER 2017           4573 (8.2G)
  CHEVROLET TRAILBLAZER 2021            5096 (9.2G)
  CHEVROLET SUBURBAN PREMIER 2019       61 (109M)
  CHEVROLET SILVERADO 1500 2020         13359 (25G)
  CHEVROLET EQUINOX NO ACC              574 (1022M)
  CHEVROLET BOLT EV NO ACC              919 (1.5G)
  CHEVROLET BOLT EUV 2022               6160 (12G)
  CADILLAC XT4 2023                     464 (758M)
  CADILLAC ESCALADE ESV 2019            1 (4.0K)
  CADILLAC ESCALADE 2017                24 (38M)
  BUICK LACROSSE 2017                   96 (172M)
ford                                    1829 (2.9G)
  MURPHY CAR                            2 (2.6M)
  FORD MAVERICK 1ST GEN                 883 (1.3G)
  FORD F-150 14TH GEN                   944 (1.6G)
chrysler                                17092 (30G)
  JEEP GRAND CHEROKEE V6 2018           50 (89M)
  RAM HD 5TH GEN                        853 (1.6G)
  RAM 1500 5TH GEN                      14249 (25G)
  CHRYSLER PACIFICA HYBRID 2019         897 (1.5G)
  CHRYSLER PACIFICA 2018                1042 (1.7G)
\`\`\`"
postface="Last updated $(date +'%B %d, %Y')"

# hours of data: 1 minute per lat file
lat_file_hours=$((lat_file_count/60))
# round to one decimal place
lat_file_hours=$(printf "%.1f" $lat_file_hours)

lat_file_total="(83718 total; $lat_file_hours hours)"
# Initialize table of contents for all directories
table_of_contents="## Table of Contents
- [1 History and future of data-driven controls improvements](https://github.com/twilsonco/openpilot/blob/log-info/sec/1%20History%20and%20future%20of%20data-driven%20controls%20improvments.md)
- [2 Understanding the NNFF model and plots](https://github.com/twilsonco/openpilot/blob/log-info/sec/2%20Understanding%20the%20NNFF%20model%20and%20plots.md)
- [3 Community vehicle log counts](#current-counts-of-collected-logs) $lat_file_total\n"

# Initialize README body
readme_body=""

mkdir -p "../thumbnails"
rm -rf "../thumbnails/*"
cd "../data"
# Loop through all subdirectories in current directory
dir_i=0
for dir in *; do
  dir_i=$((dir_i+1))
  echo `pwd`
  echo `ls`
  # Encode the directory name for use in a URL
  encoded_dirname=$(echo "$dir" | sed 's/ /%20/g')
  # dirname as lowercase with spaces changed to hyphens
  dirname_hyphen=$(echo "$dir" | sed 's/ /-/g' | tr '[:upper:]' '[:lower:]')
  # Get the directory name without the trailing slash
  dirname=$(basename "$dir")
  # Initialize counter for images in current row
  row_counter=0
  # Initialize table of contents for this directory
  section_table_of_contents="## $dirname\n"
  # Initialize table of images for this directory
  # Initialize counter
  count=1
  row=""
  # Initialize section table
  section_table=""
  table_body="| 🛣️ | 🚗 |\n| --- | --- |\n"
  # Loop through all image files in this directory
  cd "$dir"
  is_ab_dir=false
  files=(./*.png ./*.gif)
  for file in "${files[@]}"; do
    if [[ "$file" == *"*"* ]]; then
      continue
    fi
    echo "Processing $file..."
    # If file ends in "-a.png" or "-b.png", set is_ab_dir to true
    if [[ $file == *"-a.png" || $file == *"-b.png" ]]; then
      is_ab_dir=true
    fi
    if [[ $file == *"png" ]]; then
      ext=".png"
    else
      ext=".gif"
    fi
    # Get the file name without the extension
    thumbfile="${file%%.png}_${dir_i}_thumbnail.jpg"
    filename=$(basename "$file" $ext)
    thumbfilename=$(basename "$thumbfile" .jpg)
    # Generate thumbnail 
    ffmpeg -y -i "$file" -filter:v scale=400:-2 "../../thumbnails/$thumbfile"
    # Encode the filename for use in a URL
    encoded_filename=$(echo "$file" | sed 's/ /%20/g')
    img_url="https://raw.github.com/twilsonco/openpilot/log-info/data/$encoded_dirname/$encoded_filename"
    thumb_url="$img_url"
    if [[ "$file" == *"png" ]]; then
      encoded_thumbfilename=$(echo "$thumbfile" | sed 's/ /%20/g')
      # Append a new cell to the current row with the image and its file name
      thumb_url="https://raw.github.com/twilsonco/openpilot/log-info/thumbnails/$encoded_thumbfilename"
    fi
    cell="| [$filename](#table-of-contents) ([full image]($img_url)) ![$filename]($thumb_url)"
    # Increment row counter
    row_counter=$((row_counter+1))
    # If we've reached the end of a row, add a new line to the table
    if [ $row_counter -eq 2 ]; then
      cell="$cell|\n"
      row_counter=0
    fi
    table_body="$table_body$cell"
    # Add to first or second column
    if [ $count -eq 1 ]; then
      row="| [${filename}]($img_url) | "
    else
      row="$row [${filename}]($img_url) |\n"
      count=0
      # Add row to table
      section_table+="$row"
    fi

    # Increment counter
    count=$((count + 1))

  done
  cd ..
  
  if [ "$is_ab_dir" = true ] ; then
    section_table="| Lateral accel response | Error/jerk/roll responses |\n|-|-|\n$section_table"
  else
    section_table="| Car | Car |\n|-|-|\n$section_table"
  fi

  # Combine table of contents and table of images into a single table for this directory
  section_table="\n$section_table_of_contents\n\n$section_table\n\n$table_body"

  # Append the table for this directory to the README body
  # readme_body="$readme_body$section_table"
  cd ..
  mkdir -p sec
  echo "$section_table" > sec/"$dir".md
  encoded_sectionname=$(echo "$dir".md | sed 's/ /%20/g')
  section_url="https://github.com/twilsonco/openpilot/blob/log-info/sec/$encoded_sectionname"
  cd "data"

  # Add a new entry to the table of contents for all directories
  file_count=$(ls -1q "$dir"/*$ext | wc -l)
  # divide by two if is_ab_dir
  if [ "$is_ab_dir" = true ] ; then
    file_count=$((file_count/2))
  fi
  table_of_contents="$table_of_contents- [$dirname]($section_url) ($file_count cars)\n"
done
cd ..

# Combine table of contents for all directories into a single table
table_of_contents="$table_of_contents\n"

# Combine preface, table of contents, and README body into README.md
echo "$preface\n\n$table_of_contents\n\n$lat_file_list\n$readme_body\n$postface" > README.md