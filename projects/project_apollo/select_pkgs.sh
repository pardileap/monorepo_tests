# #!/bin/bash
# set -e

# # Read the folders from the JSON file
# folders=$(jq -r '.libs{}' libs_requirements.json)

# # Create a directory in the Docker image to copy the folders
# for folder in $folders; do
#   echo "Link lib: $folder"
#   # ln -s ../../../libs/modules/$folder libs/$folder  
# done
# # mkdir -p /ros2_ws/src/

# # # Loop through each folder and copy it to the Docker image
# # for folder in $folders; do
# #   echo "Copying folder: $folder"
# #   cp -r "$folder" /ros2_ws/src/
# # done
# # echo "Packages copied successfully!"
#!/bin/bash

# Ensure jq is installed
if ! command -v jq &> /dev/null; then
    echo "jq is not installed. Please install jq to use this script."
    exit 1
fi

# Read JSON file
JSON_FILE="libs_requirements.json"

# Check if JSON file exists
if [[ ! -f "$JSON_FILE" ]]; then
    echo "File $JSON_FILE does not exist."
    exit 1
fi

# Extract all libraries from the JSON file
libs=$(jq -r '.libs | to_entries[] | .key' "$JSON_FILE")

# Loop through each library and extract details
for lib in $libs; do
    version=$(jq -r ".libs[\"$lib\"].version" "$JSON_FILE")
    type=$(jq -r ".libs[\"$lib\"].type" "$JSON_FILE")

    global_path=$(pwd)
    # strip everything after monorepo_test exclude monorepo_test
    global_path=${global_path%monorepo_tests*}monorepo_tests
    echo $global_path

    ln -s $global_path/libs/$type/$lib libs/$type/$lib  
    # Print the extracted details
    echo "Library: $lib"
    echo "  Version: $version"
    echo "  Type: $type"
    echo "---------------------"
done