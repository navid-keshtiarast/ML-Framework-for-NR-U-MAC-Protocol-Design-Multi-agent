#!/bin/bash

# Usage: ./parallel_evaluation_universal.sh <SCENARIO>
# Example: ./parallel_evaluation_universal.sh CTDE

SCENARIO=$1

if [[ -z "$SCENARIO" ]]; then
    echo "Usage: $0 <SCENARIO> (CTCE, DTDE, CTDE)"
    exit 1
fi

# Define agent and rllib paths based on the scenario
case $SCENARIO in
    "CTCE")
        AGENT="central"
        RLLIB_DIR=""
        OUTPUT_BASE=""
        ;;
    "DTDE")
        AGENT="multi --separate_agent_nns"
        RLLIB_DIR=""
        OUTPUT_BASE=""
        ;;
    "CTDE")
        AGENT="multi"
        RLLIB_DIR=""
        OUTPUT_BASE=""
        ;;
    *)
        echo "Invalid scenario. Use CTCE, DTDE, or CTDE."
        exit 1
        ;;
esac

# Define configurations
TRAFFIC_TYPES=("BURST" "UDP_CBR")
DISTRIBUTIONS=("high" "mid" "low" "random")
AP_NUMS=(2 4 6)

declare -A UDP_LAMBDAS
UDP_LAMBDAS["high"]="2290 1710 2340 1250 2780 1200"
UDP_LAMBDAS["mid"]="870 930 620 580 590 610"
UDP_LAMBDAS["low"]="20 130 80 460 70 260"
UDP_LAMBDAS["random"]="380 2360 730 2560 2040 1340"

# Run simulations sequentially for each distribution
for DIST in "${DISTRIBUTIONS[@]}"; do
    echo "Running simulations for distribution: $DIST"
    
    # Run parallel jobs for different traffic types and AP numbers within the same distribution
    for TRAFFIC in "${TRAFFIC_TYPES[@]}"; do
        for AP in "${AP_NUMS[@]}"; do
            OUTPUT_DIR="$OUTPUT_BASE/dist_${DIST}_${TRAFFIC,,}"
            UDP_VALUES=${UDP_LAMBDAS[$DIST]}

            CMD="python3 evaluation.py --agent $AGENT --ap_nums $AP --outputDir $OUTPUT_DIR --rllibDir $RLLIB_DIR --trafficType=$TRAFFIC --udpLambda1=$(echo $UDP_VALUES | cut -d' ' -f1) --udpLambda2=$(echo $UDP_VALUES | cut -d' ' -f2) --udpLambda3=$(echo $UDP_VALUES | cut -d' ' -f3) --udpLambda4=$(echo $UDP_VALUES | cut -d' ' -f4) --udpLambda5=$(echo $UDP_VALUES | cut -d' ' -f5) --udpLambda6=$(echo $UDP_VALUES | cut -d' ' -f6)"

            echo "Running: $CMD"
            eval $CMD &  # Run command in background
        done
    done

    # Wait for all background jobs for the current distribution to finish before moving to the next distribution
    wait
done

echo "All evaluations completed for scenario: $SCENARIO."

