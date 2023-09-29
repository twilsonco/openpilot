#!/bin/sh

# Define preface and postface strings
preface="![intro_vid](https://github.com/twilsonco/openpilot/blob/log-info/img/comma-steering-control-vid.gif?raw=true)

# Improving controls with log data

Provide feedback on lateral performance here: https://forms.gle/dtGQrY3TSkyQo6nu7

Here's plots of the [comma-steering-control](https://github.com/commaai/comma-steering-control) dataset and the data I've collected so far from the community.
Community log collection is in a limited state; only cars not in the comma-steering-control dataset will be collected for the time being.
I'll update here periodically so log contributors can see which speeds/angles need to be filled out.

Head to the Comma, Community, or SunnyPilot Discord server #tuning (or tuning-nnff) channels if you want to contribute or learn more!"
lat_file_count=131173
lat_file_list="## Current counts of collected logs (list updated Aug 27, 2023)\n\n
\`\`\`
volkswagen                              9746 (15G)
  VOLKSWAGEN ATLAS 1ST GEN              1008 (1.6G)
  VOLKSWAGEN TIGUAN 2ND GEN             471 (730M)
  VOLKSWAGEN SHARAN 2ND GEN             56 (90M)
  VOLKSWAGEN POLO 6TH GEN               525 (805M)
  VOLKSWAGEN PASSAT NMS                 1471 (2.1G)
  VOLKSWAGEN PASSAT 8TH GEN             540 (872M)
  VOLKSWAGEN JETTA 7TH GEN              345 (549M)
  VOLKSWAGEN GOLF 7TH GEN               744 (1.1G)
  VOLKSWAGEN GOLF 6TH GEN               34 (46M)
  SKODA KODIAQ 1ST GEN                  3323 (5.0G)
  SKODA KAROQ 1ST GEN                   572 (822M)
  AUDI A3 3RD GEN                       656 (921M)
toyota                                  21577 (32G)
  TOYOTA SIENNA 2018                    50 (76M)
  TOYOTA RAV4 HYBRID 2022               8 (12M)
  TOYOTA RAV4 HYBRID 2019               4437 (6.6G)
  TOYOTA RAV4 2019                      1191 (1.9G)
TOYOTA PRIUS v 2017                     1946 (3.0G)
  TOYOTA PRIUS TSS2 2021                5008 (7.6G)
  TOYOTA PRIUS 2017                     2421 (3.6G)
  TOYOTA HIGHLANDER HYBRID 2020         10 (15M)
  TOYOTA COROLLA TSS2 2019              544 (739M)
  TOYOTA COROLLA HYBRID TSS2 2019       2508 (3.5G)
  TOYOTA COROLLA 2017                   2025 (3.0G)
  TOYOTA CAMRY HYBRID 2021              1111 (1.6G)
  LEXUS RX 2020                         95 (139M)
  LEXUS ES 2019                         222 (317M)
subaru                                  2170 (3.2G)
  SUBARU OUTBACK 6TH GEN                459 (626M)
  SUBARU LEGACY 7TH GEN                 500 (750M)
  SUBARU LEGACY 2015 - 2018             1197 (1.9G)
  SUBARU IMPREZA SPORT 2020             14 (25M)
nissan                                  986 (1.5G)
  NISSAN LEAF 2018                      986 (1.5G)
mazda                                   3583 (5.1G)
  MAZDA 3                               22 (32M)
  MAZDA 3 2014                          1 (1.0K)
  MAZDA CX-5                            1367 (2.1G)
  MAZDA CX-9                            1293 (1.8G)
  MAZDA CX-5 2022                       22 (38M)
  MAZDA 3 2019                          877 (1.3G)
hyundai                                 33094 (50G)
  KIA SPORTAGE 5TH GEN                  3078 (4.6G)
  KIA NIRO HYBRID 2ND GEN               373 (593M)
  KIA NIRO HYBRID 2021                  2881 (4.4G)
  KIA NIRO HYBRID 2019                  75 (113M)
  KIA NIRO EV 2020                      1883 (2.8G)
  KIA EV6 2022                          4682 (7.0G)
  KIA CEED INTRO ED 2019                6912 (9.8G)
  HYUNDAI TUCSON HYBRID 4TH GEN         4 (4.5M)
  HYUNDAI SONATA HYBRID 2021            249 (364M)
  HYUNDAI SONATA 2020                   2841 (4.3G)
  HYUNDAI SANTA FE HYBRID 2022          5734 (8.9G)
  HYUNDAI SANTA FE 2022                 1 (512)
  HYUNDAI SANTA FE 2019                 614 (959M)
  HYUNDAI PALISADE 2020                 831 (1.2G)
  HYUNDAI KONA ELECTRIC 2022            122 (195M)
  HYUNDAI IONIQ HYBRID 2017-2019        12 (16M)
  HYUNDAI IONIQ 5 2022                  1228 (1.9G)
  HYUNDAI GENESIS 2015-2016             113 (155M)
  GENESIS GV70 1ST GEN                  1304 (2.0G)
  GENESIS (DH)                          156 (232M)
honda                                   5305 (7.6G)
  HONDA RIDGELINE 2017                  3078 (4.4G)
  HONDA HR-V 2023                       126 (183M)
  HONDA HRV                             1 (512)
  HONDA CLARITY 2018                    356 (530M)
  HONDA CIVIC (BOSCH) 2019              506 (770M)
  HONDA CIVIC 2022                      22 (34M)
  HONDA ACCORD 2018                     1215 (1.8G)
gm                                      38762 (57G)
  GMC ACADIA DENALI 2018                6493 (9.5G)
  CHEVROLET VOLT PREMIER 2018           9180 (14G)
  CHEVROLET VOLT PREMIER 2017           3483 (5.0G)
  CHEVROLET TRAILBLAZER 2021            3292 (5.4G)
  CHEVROLET SUBURBAN PREMIER 2019       58 (90M)
  CHEVROLET SILVERADO 1500 2020         8516 (13G)
  CHEVROLET EQUINOX NO ACC              571 (769M)
  CHEVROLET BOLT EV NO ACC              916 (1.3G)
  CHEVROLET BOLT EUV 2022               6159 (8.7G)
  BUICK LACROSSE 2017                   93 (131M)
ford                                    1787 (2.5G)
  MURPHY CAR                            2 (2.3M)
  FORD MAVERICK 1ST GEN                 883 (1.2G)
  FORD F-150 14TH GEN                   902 (1.4G)
chrysler                                13985 (21G)
  RAM HD 5TH GEN                        851 (1.3G)
  RAM 1500 5TH GEN                      11198 (17G)
  CHRYSLER PACIFICA HYBRID 2019         896 (1.3G)
  CHRYSLER PACIFICA 2018                1039 (1.5G)

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