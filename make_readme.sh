#!/bin/sh

# Define preface and postface strings
preface="# Improving controls with log data

Here's plots of the [comma-steering-control](https://github.com/commaai/comma-steering-control) dataset and the data I've collected so far from the community.
Community log collection is in a limited state; only cars not in the comma-steering-control dataset will be collected for the time being.
I'll update here periodically so log contributors can see which speeds/angles need to be filled out.

Head to the Comma, Community, or SunnyPilot Discord server #tuning (or tunning-nnff) channels if you want to contribute or learn more!"
lat_file_total="(83718 total)"
lat_file_count="## Current counts of collected logs\n\n
\`\`\`
nissan                                  598 (691M)
  NISSAN LEAF 2018                      598 (691M)
mazda                                   1308 (1.4G)
  MAZDA CX-5 2022                       15 (21M)
  MAZDA CX-9                            1293 (1.4G)
subaru                                  1740 (2.1G)
  SUBARU LEGACY 2015 - 2018             774 (975M)
  SUBARU IMPREZA SPORT 2020             7 (9.4M)
  SUBARU OUTBACK 6TH GEN                459 (503M)
  SUBARU LEGACY 7TH GEN                 500 (603M)
ford                                    1787 (2.0G)
  FORD F-150 14TH GEN                   902 (1.1G)
  FORD MAVERICK 1ST GEN                 883 (955M)
  MURPHY CAR                            2 (1.9M)
volkswagen                              8379 (9.8G)
  VOLKSWAGEN JETTA 7TH GEN              345 (443M)
  SKODA KODIAQ 1ST GEN                  3172 (3.8G)
  SKODA KAROQ 1ST GEN                   572 (661M)
  VOLKSWAGEN PASSAT 8TH GEN             378 (494M)
  VOLKSWAGEN TIGUAN 2ND GEN             471 (588M)
  VOLKSWAGEN SHARAN 2ND GEN             56 (73M)
  VOLKSWAGEN POLO 6TH GEN               525 (649M)
  VOLKSWAGEN PASSAT NMS                 1459 (1.7G)
  VOLKSWAGEN GOLF 7TH GEN               744 (859M)
  AUDI A3 3RD GEN                       656 (737M)
toyota                                  13242 (16G)
  TOYOTA SIENNA 2018                    13 (16M)
  TOYOTA COROLLA 2017                   2 (612K)
  TOYOTA HIGHLANDER HYBRID 2020         10 (12M)
  TOYOTA RAV4 2019                      1114 (1.4G)
  LEXUS RX 2020                         95 (112M)
  TOYOTA PRIUS TSS2 2021                603 (741M)
  LEXUS ES 2019                         46 (54M)
  TOYOTA PRIUS v 2017                     1489 (1.9G)
  TOYOTA PRIUS 2017                     1606 (1.9G)
  TOYOTA CAMRY HYBRID 2021              1022 (1.2G)
  TOYOTA COROLLA HYBRID TSS2 2019       2261 (2.6G)
  TOYOTA COROLLA TSS2 2019              544 (594M)
  TOYOTA RAV4 HYBRID 2019               4437 (5.3G)
hyundai                                 17828 (22G)
  KIA NIRO HYBRID 2021                  367 (446M)
  KIA NIRO HYBRID 2ND GEN               190 (243M)
  HYUNDAI SANTA FE HYBRID 2022          1375 (1.8G)
  KIA NIRO HYBRID 2019                  75 (91M)
  KIA NIRO EV 2020                      246 (282M)
  KIA CEED INTRO ED 2019                2371 (2.6G)
  HYUNDAI SANTA FE 2019                 614 (771M)
  HYUNDAI KONA ELECTRIC 2022            116 (148M)
  HYUNDAI IONIQ HYBRID 2017-2019        12 (13M)
  HYUNDAI SANTA FE 2022                 1 (512)
  GENESIS GV70 1ST GEN                  1304 (1.6G)
  GENESIS (DH)                          156 (186M)
  KIA SPORTAGE 5TH GEN                  2049 (2.5G)
  HYUNDAI IONIQ 5 2022                  1223 (1.5G)
  KIA EV6 2022                          4659 (5.6G)
  HYUNDAI SONATA 2020                   2331 (2.9G)
  HYUNDAI PALISADE 2020                 738 (871M)
honda                                   4141 (4.7G)
  HONDA CIVIC (BOSCH) 2019              346 (419M)
  HONDA CLARITY 2018                    2 (1.9M)
  HONDA ACCORD 2018                     565 (681M)
  HONDA HR-V 2023                       126 (147M)
  HONDA HRV                             1 (512)
  HONDA CIVIC 2022                      22 (27M)
  HONDA RIDGELINE 2017                  3078 (3.5G)
chrysler                                7046 (8.1G)
  RAM HD 5TH GEN                        851 (1.1G)
  RAM 1500 5TH GEN                      4260 (4.9G)
  CHRYSLER PACIFICA HYBRID 2019         896 (1.1G)
  CHRYSLER PACIFICA 2018                1039 (1.2G)
gm                                      27471 (33G)
  CHEVROLET TRAILBLAZER 2021            801 (991M)
  CHEVROLET BOLT EV NO ACC              916 (990M)
  GMC ACADIA DENALI 2018                5843 (6.8G)
  CHEVROLET SUBURBAN PREMIER 2019       58 (72M)
  CHEVROLET SILVERADO 1500 2020         4581 (5.5G)
  CHEVROLET BOLT EUV 2022               5876 (6.7G)
  BUICK LACROSSE 2017                   93 (105M)
  CHEVROLET VOLT PREMIER 2018           6274 (8.4G)
  CHEVROLET VOLT PREMIER 2017           3028 (3.5G)
\`\`\`"
postface="Last updated $(date +'%B %d, %Y')"

# Initialize table of contents for all directories
table_of_contents="## Table of Contents\n- [1 Community vehicle log counts](#current-counts-of-collected-logs) $lat_file_total\n"

# Initialize README body
readme_body=""

mkdir -p "thumbnails"
rm -rf "thumbnails/*"
cd "data"
# Loop through all subdirectories in current directory
for dir in *; do
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
  section_table="| Car | Car |\n|-|-|\n"
  table_body="| 🛣️ | 🚗 |\n| --- | --- |\n"
  # Loop through all image files in this directory
  cd "$dir"
  for file in *.png; do
    echo "Processing $file..."
    # Get the file name without the extension
    thumbfile="${file%%.png}_thumbnail.jpg"
    filename=$(basename "$file" .png)
    thumbfilename=$(basename "$thumbfile" .png)
    # Generate thumbnail 
    # ffmpeg -y -i "$file" -filter:v scale=400:-2 "../../thumbnails/$thumbfile"
    # Encode the filename for use in a URL
    encoded_filename=$(echo "$file" | sed 's/ /%20/g')
    encoded_thumbfilename=$(echo "$thumbfile" | sed 's/ /%20/g')
    # Append a new cell to the current row with the image and its file name
    img_url="https://github.com/twilsonco/openpilot/blob/log-info/data/$encoded_dirname/$encoded_filename?raw=true"
    thumb_url="https://github.com/twilsonco/openpilot/blob/log-info/thumbnails/$encoded_thumbfilename?raw=true"
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

  # Combine table of contents and table of images into a single table for this directory
  section_table="\n$section_table_of_contents\n\n$section_table\n\n$table_body"

  # Append the table for this directory to the README body
  readme_body="$readme_body$section_table"

  # Add a new entry to the table of contents for all directories
  table_of_contents="$table_of_contents- [$dirname](#$dirname_hyphen) ($(ls -1q "$dir"/*.png | wc -l) cars)\n"
done
cd ..

# Combine table of contents for all directories into a single table
table_of_contents="$table_of_contents\n"

# Combine preface, table of contents, and README body into README.md
echo "$preface\n\n$table_of_contents\n\n$lat_file_count\n$readme_body\n$postface" > README.md