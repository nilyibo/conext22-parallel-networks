#!/bin/zsh


# This determines the switch radix used in figure 7 in equivalent fat-tree
# NOTE: can increase to larger radix, but runtime increase exponentially
PARAM_PART1_RADIX=8

# Can increase this to reduce standard deviation
NUM_EXPERIMENTS=5

set -e
# set -x

cd "$(dirname "$0")"

echo "This script reproduces the bulk traffic throughput result in section 5.1.1, i.e. the data need for figure 6-8."
echo "You can use the corresponding jupyter notebook to plot the data once this completes."

SIM_DIR="$(realpath ../sim)"
SCRIPT_DIR="$SIM_DIR/scripts"
BUILD_DIR="$SIM_DIR/build"
GRAPH_DIR="$SIM_DIR/graphs"
TM_DIR="$SIM_DIR/traffic_matrix"

source "$SCRIPT_DIR/util.sh"

ROOT_DIR="$(realpath ../data/1.synthetic/1.bulk)"
mkdir -p "$ROOT_DIR"

# ******************* Part 1 *******************

subdir="$ROOT_DIR/1.network-core.all-to-all"
mkdir -p "$subdir"

# only this experinent is at rack-level, i.e. without servers
pushd "$SIM_DIR"
ln -sfn ./pnet.old ./pnet
popd

pushd "$SIM_DIR/build/release"
cmake -DWITH_SERVERS=OFF -DWITH_LP_FAIR_SHARING=ON .
$SIM_DIR/scripts/build.sh
popd

sed -i 's/INCLUDE_SERVERS=.*/INCLUDE_SERVERS=false/' "$SCRIPT_DIR/run-topology.sh"

MAX_NETWORKS=8

function run_experiment() {
    local radix=$1
    local num_networks=$2
    local single_or_parallel=$3
    local topology_index=$4
    local topology=$5
    local routing="$6"
    local traffic_type="$7"
    local mode=$8
    local topology_indices=1
    if [ "$topology" = "jellyfish" ]; then
        if [ $num_networks -eq 1 ]; then
            topology_indices=$(seq 1 $MAX_NETWORKS)
        else
            [ "$topology_index" = "all" ] && topology_indices="all" || topology_indices=$(seq 1 $num_networks)
        fi
    fi
    for topology_index in $(echo $topology_indices); do
        for exp_num in $(seq 1 $NUM_EXPERIMENTS); do
            cmd="./run-topology.sh $mode $topology $radix $traffic_type exp$exp_num $num_networks $(echo $routing) any $topology_index $single_or_parallel"
            echo "+$cmd"
            isem $(echo $cmd)
        done
    done
}

function run_all_lp()
{
    local topology=$1
    local routing="$2"
    local traffic_type="$3"
    local mode="$4"
    for k in $PARAM_PART1_RADIX; do
        for num_networks in 1 2 4 8; do
            # LP only, 2x serial high-bw == 2 * serial low_bw == 2x parallel homogeneous
            # LP-routes/packet: 2x serial high-bw != 2 * serial low_bw != 2x parallel homogeneous
            if [ $num_networks -eq 1 ]; then
                # serial low-bw
                run_experiment "$k" "$num_networks" "single" "" "$topology" "$routing" "$traffic_type" "$mode"
            else	# $num_networks -ge 1
                # serial high-bw
                [ "$mode" = "lp-routes" -o "$mode" = "packet" ] && run_experiment "$k" "$num_networks" "single" "" "$topology" "$routing" "$traffic_type" "$mode"
                # parallel homogeneous
                [ "$mode" = "lp-routes" -o "$mode" = "packet" ] && run_experiment "$k" "$num_networks" "parallel" "" "$topology" "$routing" "$traffic_type" "$mode"
                # parallel heterogeneous
                [ "$topology" = "jellyfish" ] && run_experiment "$k" "$num_networks" "parallel" "all" "$topology" "$routing" "$traffic_type" "$mode"
            fi
        done
    done
    isemwait
}

echo "Running part 1 experiments in parallel ..."
pushd "$SCRIPT_DIR"
( setopt +o nomatch; rm -rf $BUILD_DIR/*exp*)
mode="lp"
traffic_type="all-to-all"
topology="jellyfish"
routing="mptcp ksp 8"		# KSP, 8-way
run_all_lp "$topology" "$routing" "$traffic_type" "$mode"
popd

echo "Pre-processing data ..."
pushd "$BUILD_DIR"
grep -F 'objective' **/gurobi.log > "$subdir/result.jellyfish.lp.all-to-all.txt"
rm -r *exp*
popd

echo "Part 1 done"


# ******************* Part 2 *******************

subdir="$ROOT_DIR/2.server-to-server"
mkdir -p "$subdir"

# restore simulator to include servers
sed -i 's/INCLUDE_SERVERS=.*/INCLUDE_SERVERS=true/' "$SCRIPT_DIR/run-topology.sh"
pushd "$SIM_DIR/build/release"
cmake -DWITH_SERVERS=ON -DWITH_LP_FAIR_SHARING=ON .
$SIM_DIR/scripts/build.sh -c
popd

echo "Running part 2 experiments in parallel ..."
pushd "$SCRIPT_DIR"
( setopt +o nomatch; rm -rf $BUILD_DIR/*exp*)
mode="lp-routes"
for traffic_type in "all-to-all" "permutation"; do
    echo "Running LP solver ..."
    run_all_lp "jellyfish" "mptcp ksp 8" "$traffic_type" "$mode"
    echo "Pre-processing data ..."
    pushd "$BUILD_DIR"
    grep -F 'objective' **/gurobi.log >> "$subdir/result.jellyfish.lp-routes.$traffic_type.txt"
    rm -r *exp*
    popd

    echo "Running LP solver ..."
    run_all_lp "fat-tree" "tcp ecmp 1" "$traffic_type" "$mode"
    echo "Pre-processing data ..."
    pushd "$BUILD_DIR"
    grep -F 'objective' **/gurobi.log > "$subdir/result.fat-tree.lp-routes.$traffic_type.txt"
    rm -r *exp*
    popd
done
popd

echo "Part 2 done"

# ******************* Part 3 *******************



echo "All done. Please use eval.micro.bulk.ipynb to plot the result."
