#!/usr/bin/env python3

import os

def remove_redundant_lines(input_file_path):
  # Get the output file path by adding '_no_redundant' to the input file name
  file_name, file_ext = os.path.splitext(input_file_path)
  output_file_path = f"{file_name}_no_redundant{file_ext}"

  with open(input_file_path, 'r') as input_file, open(output_file_path, 'w') as output_file:
    prev_line = None
    count = 0
    for line in input_file:
      if prev_line is None:
        output_file.write(line.replace('\n','') + ", count\n")
      elif line.split(",")[1:] != prev_line.split(",")[1:]:
        if "Message" not in prev_line:
          output_file.write(prev_line.replace('\n','') + f", {count}\n")
          count = 0
      else:
        count += 1
      prev_line = line
      

  print(f"Processed file saved as: {output_file_path}")

if __name__ == "__main__":
  input_file_path = "/Users/haiiro/Downloads/GMlanOutput_3.csv"
  remove_redundant_lines(input_file_path)