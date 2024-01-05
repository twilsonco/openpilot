#!/bin/sh

# Define preface and postface strings
preface="[![intro_vid](https://github.com/twilsonco/openpilot/blob/log-info/img/comma-steering-control-vid.gif?raw=true)](https://youtu.be/G_SNRJCGKHs?si=4lMWhB3kcHj6Fu9E)

# Improving controls with log data

Provide feedback on lateral performance here: https://forms.gle/dtGQrY3TSkyQo6nu7

Here's plots of the [comma-steering-control](https://github.com/commaai/comma-steering-control) dataset and the data I've collected so far from the community.
Community log collection is in a limited state; only cars not in the comma-steering-control dataset will be collected for the time being.
I'll update here periodically so log contributors can see which speeds/angles need to be filled out.

Head to the Comma, Community, or SunnyPilot Discord server #tuning (or tuning-nnff) channels if you want to contribute or learn more!"
lat_file_count=168112
lat_file_list="## Current counts of collected logs\n\n
\`\`\`
volkswagen                              10365 (22G)
  VOLKSWAGEN TIGUAN 2ND GEN             480 (1.1G)
  VOLKSWAGEN SHARAN 2ND GEN             59 (106M)
  VOLKSWAGEN POLO 6TH GEN               549 (1.1G)
  VOLKSWAGEN PASSAT NMS                 1470 (2.6G)
  VOLKSWAGEN PASSAT 8TH GEN             660 (1.7G)
  VOLKSWAGEN JETTA 7TH GEN              362 (1020M)
  VOLKSWAGEN GOLF 7TH GEN               753 (1.8G)
  VOLKSWAGEN GOLF 6TH GEN               37 (51M)
  VOLKSWAGEN ATLAS 1ST GEN              1013 (2.3G)
  SKODA KODIAQ 1ST GEN                  3330 (7.1G)
  SKODA KAROQ 1ST GEN                   578 (1.1G)
  SEAT LEON 3RD GEN                     412 (815M)
  AUDI A3 3RD GEN                       662 (1.2G)
toyota                                  23568 (58G)
  TOYOTA SIENNA 2018                    56 (127M)
  TOYOTA RAV4 HYBRID 2022               14 (21M)
  TOYOTA RAV4 HYBRID 2019               4441 (11G)
  TOYOTA RAV4 2019                      1200 (3.4G)
TOYOTA PRIUS v 2017                     1370 (2.8G)
  TOYOTA PRIUS TSS2 2021                6393 (17G)
  TOYOTA PRIUS 2017                     2504 (6.6G)
  TOYOTA HIGHLANDER HYBRID 2020         16 (21M)
  TOYOTA COROLLA TSS2 2019              556 (1.3G)
  TOYOTA COROLLA HYBRID TSS2 2019       1786 (4.1G)
  TOYOTA COROLLA 2017                   3569 (8.5G)
  TOYOTA CAMRY HYBRID 2021              1116 (2.6G)
  LEXUS RX 2020                         105 (249M)
  LEXUS ES 2019                         441 (1.1G)
subaru                                  3758 (11G)
  SUBARU OUTBACK 2018 - 2019            19 (20M)
  SUBARU OUTBACK 6TH GEN                472 (1.2G)
  SUBARU OUTBACK 2015 - 2017            85 (219M)
  SUBARU LEGACY 7TH GEN                 510 (1.5G)
  SUBARU LEGACY 2015 - 2018             1244 (3.9G)
  SUBARU IMPREZA SPORT 2020             106 (305M)
  SUBARU FORESTER 2017 - 2018           1322 (3.2G)
nissan                                  1297 (2.4G)
  NISSAN LEAF 2018                      1297 (2.4G)
mazda                                   5606 (11G)
  MAZDA CX-9 2021                       24 (31M)
  MAZDA CX-9                            1301 (2.3G)
  MAZDA CX-5 2022                       115 (319M)
  MAZDA CX-5                            1377 (3.1G)
  MAZDA 3 2019                          2593 (4.7G)
  MAZDA 3 2014                          76 (141M)
  MAZDA 3                               120 (273M)
ford                                    1829 (3.0G)
  MURPHY CAR                            2 (2.9M)
  FORD MAVERICK 1ST GEN                 883 (1.5G)
  FORD F-150 14TH GEN                   944 (1.6G)
chrysler                                17522 (33G)
Dodge Durango 2020                      413 (722M)
  RAM HD 5TH GEN                        856 (1.7G)
  RAM 1500 5TH GEN                      14252 (27G)
  JEEP GRAND CHEROKEE V6 2018           56 (114M)
  CHRYSLER PACIFICA HYBRID 2019         900 (1.6G)
  CHRYSLER PACIFICA 2018                1045 (1.9G)
hyundai                                 47221 (115G)
  GENESIS G70 2020                      539 (1001M)
  HYUNDAI IONIQ ELECTRIC 2020           30 (50M)
  KIA STINGER GT2 2018                  95 (213M)
  KIA SPORTAGE 5TH GEN                  3200 (8.2G)
  KIA NIRO HYBRID 2ND GEN               377 (1.1G)
  KIA NIRO HYBRID 2021                  2908 (7.7G)
  KIA NIRO HYBRID 2019                  81 (191M)
  KIA NIRO EV 2020                      2114 (5.6G)
  KIA K8 HYBRID 1ST GEN                 1 (108K)
  KIA EV6 2022                          4438 (12G)
  KIA CEED INTRO ED 2019                10515 (25G)
  HYUNDAI TUCSON HYBRID 4TH GEN         15 (20M)
  HYUNDAI SONATA HYBRID 2021            257 (664M)
  HYUNDAI SONATA 2020                   2914 (7.6G)
  HYUNDAI SANTA FE HYBRID 2022          13356 (30G)
  HYUNDAI SANTA FE 2022                 810 (1.9G)
  HYUNDAI SANTA FE 2019                 620 (1.8G)
  HYUNDAI PALISADE 2020                 863 (2.3G)
  HYUNDAI KONA ELECTRIC 2022            985 (2.4G)
  HYUNDAI IONIQ PHEV 2020               1615 (4.1G)
  HYUNDAI IONIQ HYBRID 2017-2019        18 (26M)
  HYUNDAI IONIQ 5 2022                  1189 (3.2G)
  HYUNDAI GENESIS 2015-2016             118 (254M)
  GENESIS GV70 1ST GEN                  1 (72K)
  GENESIS (DH)                          161 (392M)
honda                                   6724 (16G)
  HONDA RIDGELINE 2017                  3360 (8.0G)
  HONDA HR-V 2023                       136 (274M)
  HONDA HRV                             1 (4.0K)
  HONDA CLARITY 2018                    1419 (3.3G)
  HONDA CIVIC (BOSCH) 2019              536 (1.3G)
  HONDA CIVIC 2022                      28 (53M)
  HONDA ACCORD 2018                     1244 (3.0G)
gm                                      50036 (91G)
  CHEVROLET BOLT EUV 2022               6160 (12G)
  CADILLAC XT4 2023                     3 (1.9M)
  CADILLAC ESCALADE ESV 2019            1 (4.0K)
  CADILLAC ESCALADE 2017                24 (38M)
  BUICK LACROSSE 2017                   96 (173M)
  GMC YUKON NO ACC                      101 (186M)
  GMC ACADIA DENALI 2018                4754 (9.4G)
  CHEVROLET VOLT PREMIER 2018           12882 (24G)
  CHEVROLET VOLT PREMIER 2017           4573 (8.2G)
  CHEVROLET TRAILBLAZER 2021            6147 (12G)
  CHEVROLET SUBURBAN PREMIER 2019       61 (110M)
  CHEVROLET SILVERADO 1500 2020         14308 (26G)
  CHEVROLET EQUINOX NO ACC              6 (3.6M)
  CHEVROLET BOLT EV NO ACC              919 (1.5G)
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