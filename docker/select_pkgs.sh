#!/bin/bash
set -e

# Read the folders from the JSON file
folders=$(jq -r '.pkgs[]' pkgs.json)

# Create a directory in the Docker image to copy the folders
mkdir -p /ros2_ws/src/

# Loop through each folder and copy it to the Docker image
for folder in $folders; do
  echo "Copying folder: $folder"
  cp -r "$folder" /ros2_ws/src/
done
echo "Packages copied successfully!"