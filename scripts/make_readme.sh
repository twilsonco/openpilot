#!/bin/sh

# Define preface and postface strings
preface="![intro_vid](https://github.com/twilsonco/openpilot/blob/log-info/img/comma-steering-control-vid.gif?raw=true)

# Improving controls with log data

Provide feedback on lateral performance here: https://forms.gle/dtGQrY3TSkyQo6nu7

Here's plots of the [comma-steering-control](https://github.com/commaai/comma-steering-control) dataset and the data I've collected so far from the community.
Community log collection is in a limited state; only cars not in the comma-steering-control dataset will be collected for the time being.
I'll update here periodically so log contributors can see which speeds/angles need to be filled out.

Head to the Comma, Community, or SunnyPilot Discord server #tuning (or tuning-nnff) channels if you want to contribute or learn more!"
lat_file_count=115919
lat_file_list="## Current counts of collected logs (list updated Aug 27, 2023)\n\n
\`\`\`
nissan                                  975 (1.9G)
  NISSAN LEAF 2018                      975 (1.9G)
mazda                                   2205 (3.1G)
  MAZDA 3 2019                          882 (1.4G)
  MAZDA CX-5 2022                       25 (37M)
  MAZDA CX-9                            1298 (1.7G)
subaru                                  2189 (3.9G)
  SUBARU LEGACY 2015 - 2018             1202 (2.3G)
  SUBARU IMPREZA SPORT 2020             18 (22M)
  SUBARU OUTBACK 6TH GEN                464 (727M)
  SUBARU LEGACY 7TH GEN                 505 (904M)
ford                                    1802 (3.3G)
  FORD F-150 14TH GEN                   907 (1.8G)
  FORD MAVERICK 1ST GEN                 888 (1.5G)
  MURPHY CAR                            7 (2.4M)
volkswagen                              8628 (15G)
  VOLKSWAGEN GOLF 6TH GEN               39 (48M)
  VOLKSWAGEN JETTA 7TH GEN              350 (768M)
  SKODA KODIAQ 1ST GEN                  3275 (5.5G)
  SKODA KAROQ 1ST GEN                   577 (905M)
  VOLKSWAGEN PASSAT 8TH GEN             445 (874M)
  VOLKSWAGEN TIGUAN 2ND GEN             476 (860M)
  VOLKSWAGEN SHARAN 2ND GEN             61 (97M)
  VOLKSWAGEN POLO 6TH GEN               530 (914M)
  VOLKSWAGEN PASSAT NMS                 1464 (2.5G)
  VOLKSWAGEN GOLF 7TH GEN               749 (1.4G)
  AUDI A3 3RD GEN                       661 (955M)
toyota                                  18142 (32G)
  TOYOTA RAV4 HYBRID 2022               13 (15M)
  TOYOTA SIENNA 2018                    42 (64M)
  TOYOTA COROLLA 2017                   1302 (2.2G)
  TOYOTA HIGHLANDER HYBRID 2020         15 (14M)
  TOYOTA RAV4 2019                      1196 (2.3G)
  LEXUS RX 2020                         100 (177M)
  TOYOTA PRIUS TSS2 2021                3263 (6.3G)
  LEXUS ES 2019                         227 (380M)
TOYOTA PRIUS v 2017                     1635 (2.9G)
  TOYOTA PRIUS 2017                     1894 (3.6G)
  TOYOTA CAMRY HYBRID 2021              1027 (1.5G)
  TOYOTA COROLLA HYBRID TSS2 2019       2437 (4.1G)
  TOYOTA COROLLA TSS2 2019              549 (892M)
  TOYOTA RAV4 HYBRID 2019               4442 (7.8G)
hyundai                                 28414 (50G)
  HYUNDAI SONATA HYBRID 2021            117 (202M)
  HYUNDAI TUCSON HYBRID 4TH GEN         9 (4.9M)
  HYUNDAI GENESIS 2015-2016             118 (209M)
  KIA NIRO HYBRID 2021                  2864 (5.2G)
  KIA NIRO HYBRID 2ND GEN               376 (764M)
  HYUNDAI SANTA FE HYBRID 2022          3662 (6.7G)
  KIA NIRO HYBRID 2019                  80 (121M)
  KIA NIRO EV 2020                      1361 (2.6G)
  KIA CEED INTRO ED 2019                5249 (8.5G)
  HYUNDAI SANTA FE 2019                 619 (1.3G)
  HYUNDAI KONA ELECTRIC 2022            121 (204M)
  HYUNDAI IONIQ HYBRID 2017-2019        17 (19M)
  HYUNDAI SANTA FE 2022                 1 (512)
  GENESIS GV70 1ST GEN                  1309 (2.3G)
  GENESIS (DH)                          161 (322M)
  KIA SPORTAGE 5TH GEN                  2815 (4.7G)
  HYUNDAI IONIQ 5 2022                  1228 (2.2G)
  KIA EV6 2022                          4687 (7.9G)
  HYUNDAI SONATA 2020                   2783 (4.9G)
  HYUNDAI PALISADE 2020                 836 (1.6G)
honda                                   5332 (8.0G)
  HONDA CIVIC (BOSCH) 2019              510 (795M)
  HONDA CLARITY 2018                    361 (577M)
  HONDA ACCORD 2018                     1220 (1.9G)
  HONDA HR-V 2023                       131 (177M)
  HONDA HRV                             1 (512)
  HONDA CIVIC 2022                      25 (34M)
  HONDA RIDGELINE 2017                  3083 (4.6G)
chrysler                                12191 (19G)
  RAM HD 5TH GEN                        856 (1.3G)
  RAM 1500 5TH GEN                      9390 (15G)
  CHRYSLER PACIFICA HYBRID 2019         901 (1.2G)
  CHRYSLER PACIFICA 2018                1044 (1.4G)
gm                                      35862 (57G)
  CHEVROLET EQUINOX NO ACC              496 (807M)
  CHEVROLET TRAILBLAZER 2021            1892 (2.5G)
  CHEVROLET BOLT EV NO ACC              921 (1.3G)
  GMC ACADIA DENALI 2018                6498 (11G)
  CHEVROLET SUBURBAN PREMIER 2019       63 (93M)
  CHEVROLET SILVERADO 1500 2020         7649 (13G)
  CHEVROLET BOLT EUV 2022               6164 (9.9G)
  BUICK LACROSSE 2017                   98 (161M)
  CHEVROLET VOLT PREMIER 2018           8597 (14G)
  CHEVROLET VOLT PREMIER 2017           3483 (5.2G)

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