#!/bin/bash

# Set the file extension
ext="-a.png"

# Check that two arguments were provided
if [ "$#" -ne 3 ]; then
  echo "Usage: $0 model_plot_folder model_folder output_folder"
  exit 1
fi

# Loop through all files in the model plot folder
for file1 in "$1"/*"$ext"; do
  # Check that file1 is a regular file 
  if [ -f "$file1" ]; then
    # Get the name of the file without the extension
    filename=$(basename "$file1" "$ext")
    # Check if the corresponding model folder exists in model folder
    model_folder="$2/$filename"
    if [ -d "$model_folder" ]; then
      # Check if the model file exists in the model folder
      model_file="$model_folder/$filename.json"
      if [ -f "$model_file" ]; then
        # Copy the model file and image file into the output folder
        echo "Copying $file1 and $model_file to $3"
        cp -f "$file1" "$3"
        cp -f "$model_file" "$3"
      else
        echo "Model file not found for $file1"
      fi
    else
      echo "Model folder not found for $file1"
    fi
  fi
done