#!/bin/bash

# Ensure the script to be called exists
SORT_SCRIPT="$(dirname "$0")/delete_spam.sh"
if [ ! -f "$SORT_SCRIPT" ]; then
    echo "Error: delete_spam.sh not found! Make sure it is in the same directory."
    exit 1
fi

# Base directory
BASE_DIR="/home/navid/ns-3.38/contrib/ai/examples/nr-ai-mac/use-gym/eval_results_delay"

# Define arrays for the different parts of the directory paths
MODES=("CTCE" "CTDE" "DTDE")
TRAFFIC_TYPES=("dist_high_burst" "dist_high_udp" "dist_mid_burst" "dist_mid_udp" "dist_low_burst" "dist_low_udp" "dist_random_burst" "dist_random_udp")
FRAMEWORKS=("ns3" "rllib")
NUMBERS=("2" "4" "6")

# Loop through all combinations and generate directory paths
for MODE in "${MODES[@]}"; do
    for TRAFFIC_TYPE in "${TRAFFIC_TYPES[@]}"; do
        for FRAMEWORK in "${FRAMEWORKS[@]}"; do
            for NUMBER in "${NUMBERS[@]}"; do
                DIR="$BASE_DIR/$MODE/$TRAFFIC_TYPE/$FRAMEWORK/$NUMBER"
                if [ -d "$DIR" ]; then
                    echo "Sorting files in: $DIR"
                    bash "$SORT_SCRIPT" "$DIR"
                else
                    echo "Warning: $DIR is not a valid directory. Skipping."
                fi
            done
        done
    done
done

echo "Automated batch sorting complete!"