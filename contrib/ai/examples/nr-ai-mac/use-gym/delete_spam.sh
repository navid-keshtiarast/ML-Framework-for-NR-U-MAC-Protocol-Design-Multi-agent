#!/bin/bash

# Set the target directory (default to current directory if not provided)
TARGET_DIR=${1:-$(pwd)}

# Ensure the directory exists
if [ ! -d "$TARGET_DIR" ]; then
    echo "Error: Directory does not exist."
    exit 1
fi

# Get a list of files sorted by modification time (newest first)
mapfile -t FILES < <(find "$TARGET_DIR" -maxdepth 1 -type f -printf "%T@ %p\n" | sort -nr | cut -d' ' -f2-)

# Count total files
FILE_COUNT=${#FILES[@]}

# If more than 10 files exist, delete the newest ones to keep only the oldest 10
if [ "$FILE_COUNT" -gt 10 ]; then
    for ((i=0; i<FILE_COUNT-10; i++)); do
        echo "Deleting: ${FILES[i]}"
        rm -f "${FILES[i]}"
    done
fi

echo "Sorting complete! Kept the 10 oldest files, deleted the rest."
