#!/bin/sh

# Define preface and postface strings
preface="# Improving controls with log data\n\nHere's the data I've collected so far from the community. I'll update here periodically so you can see which speeds/angles need to be filled out.\n\nHead to the SunnyPilot Discord server #tuning-nnff channel if you want to contribute or learn more!"
postface="Last updated $(date +'%B %d, %Y')"

# Initialize table of contents for all directories
table_of_contents="## Table of Contents\n"

# Initialize README body
readme_body=""

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
  table_body="| 🛣️ | 🚗 |\n| --- | --- |\n"
  # Loop through all image files in this directory
  cd "$dir"
  for file in *.png; do
    echo "Processing $file..."
    # Get the file name without the extension
    filename=$(basename "$file" .png)
    # Encode the filename for use in a URL
    encoded_filename=$(echo "$file" | sed 's/ /%20/g')
    # Append a new cell to the current row with the image and its file name
    cell="| [$filename](#table-of-contents)  ![$filename](https://github.com/twilsonco/openpilot/blob/log-info/data/$encoded_dirname/$encoded_filename?raw=true)"
    # Increment row counter
    row_counter=$((row_counter+1))
    # If we've reached the end of a row, add a new line to the table
    if [ $row_counter -eq 2 ]; then
      cell="$cell|\n"
      row_counter=0
    fi
    table_body="$table_body$cell"
  done
  cd ..

  # Combine table of contents and table of images into a single table for this directory
  section_table="\n$section_table_of_contents\n\n$table_body"

  # Append the table for this directory to the README body
  readme_body="$readme_body$section_table"

  # Add a new entry to the table of contents for all directories
  table_of_contents="$table_of_contents- [$dirname](#$dirname_hyphen) ($(ls -1q "$dir"/*.png | wc -l) cars)\n"
done
cd ..

# Combine table of contents for all directories into a single table
table_of_contents="$table_of_contents\n"

# Combine preface, table of contents, and README body into README.md
echo "$preface\n\n$table_of_contents\n$readme_body\n$postface" > README.md