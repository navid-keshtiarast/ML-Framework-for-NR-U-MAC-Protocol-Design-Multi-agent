# Base directories
BASE_OUTPUT_DIR=""
RLLIB_DIR=""

# Traffic types
TRAFFIC_TYPES=("BURST" "UDP_CBR")

# AP numbers
AP_NUMS=(2 4 6)

# Lambda values for different distributions
declare -A LAMBDA_VALUES
LAMBDA_VALUES["high"]="--udpLambda1=2290 --udpLambda2=1710 --udpLambda3=2340 --udpLambda4=1250 --udpLambda5=2780 --udpLambda6=1200"
LAMBDA_VALUES["mid"]="--udpLambda1=870 --udpLambda2=930 --udpLambda3=620 --udpLambda4=580 --udpLambda5=590 --udpLambda6=610"
LAMBDA_VALUES["low"]="--udpLambda1=20 --udpLambda2=130 --udpLambda3=80 --udpLambda4=460 --udpLambda5=70 --udpLambda6=260"
LAMBDA_VALUES["random"]="--udpLambda1=380 --udpLambda2=2360 --udpLambda3=730 --udpLambda4=2560 --udpLambda5=2040 --udpLambda6=1340"

# Function to run evaluation
run_evaluation() {
    local traffic_type=$1
    local ap_num=$2
    local output_dir=$3
    local lambda_values=$4

    python3 evaluation.py --agent multi --ap_nums $ap_num --outputDir $output_dir --rllibDir $RLLIB_DIR --trafficType=$traffic_type $lambda_values
}

# Loop through each lambda value group
for distribution in "${!LAMBDA_VALUES[@]}"; do
    # Start processes for all traffic types and AP numbers in parallel
    for traffic_type in "${TRAFFIC_TYPES[@]}"; do
        # Adjust traffic type for output directory
        formatted_traffic_type=$(echo "$traffic_type" | tr '[:upper:]' '[:lower:]')  # Convert to lowercase
        [[ "$formatted_traffic_type" == "udp_cbr" ]] && formatted_traffic_type="udp"  # Remove '_cbr'

        # Construct the output directory
        OUTPUT_DIR="$BASE_OUTPUT_DIR/dist_${distribution}_${formatted_traffic_type}"

        for ap_num in "${AP_NUMS[@]}"; do
            run_evaluation $traffic_type $ap_num $OUTPUT_DIR "${LAMBDA_VALUES[$distribution]}" &
        done
    done

    # Wait for all parallel jobs of this lambda group to finish before moving to the next
    wait
done