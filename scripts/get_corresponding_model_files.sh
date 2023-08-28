#!/bin/bash

# Set the file extension
ext="png"

# Check that two arguments were provided
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 folder1 folder2"
  exit 1
fi

# Loop through all files in folder1
for file1 in "$1"/*."$ext"; do
  # Check that file1 is a regular file
  if [ -f "$file1" ]; then
    # Get the name of the file without the extension
    filename=$(basename "$file1" ."$ext")
    # Check if the corresponding model folder exists in folder2
    model_folder="$2/$filename"
    if [ -d "$model_folder" ]; then
      # Check if the model file exists in the model folder
      model_file="$model_folder/$filename.json"
      if [ -f "$model_file" ]; then
        # Copy the model file into folder1
        cp -f "$model_file" "$1"
      else
        echo "Model file not found for $file1"
      fi
    else
      echo "Model folder not found for $file1"
    fi
  fi
done