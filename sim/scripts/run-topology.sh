#!/bin/bash

cd "$(dirname "$0")"

source ./util.sh

DEFAULT_TOPOLOGY="jellyfish"
DEFAULT_SWITCH_RADIX=8
DEFAULT_TRAFFIC_TYPE="all-to-all"
DEFAULT_EXP_SEQ="exp1"
DEFAULT_NUM_NETWORKS=2
DEFAULT_TRANSPORT_TYPE="tcp"
DEFAULT_ROUTING_PROTOCOL="ecmp"
DEFAULT_K_SHORTEST_PATHS=1
# Choose from "any", "single", "shortest" or "round-robin"
DEFAULT_SCHEDULING_MODE="any"

TOPOLOGY=${2:-DEFAULT_TOPOLOGY}
SWITCH_RADIX=${3:-DEFAULT_SWITCH_RADIX}
TRAFFIC_TYPE=${4:-DEFAULT_TRAFFIC_TYPE}
EXP_SEQ=${5:-DEFAULT_EXP_SEQ}
NUM_NETWORKS=${6:-DEFAULT_NUM_NETWORKS}
MAX_NETWORKS=8
TRANSPORT_TYPE=${7:-DEFAULT_TRANSPORT_TYPE}
ROUTING_PROTOCOL=${8:-DEFAULT_ROUTING_PROTOCOL}
K_SHORTEST_PATHS=${9:-DEFAULT_K_SHORTEST_PATHS}
SCHEDULING_MODE=${10:-DEFAULT_SCHEDULING_MODE}
INCLUDE_SERVERS=true
FIRST_FIT=false
LP_FAIR_SHARING=false
TH_W=3
TH_S=$K_SHORTEST_PATHS
SPREADING_POLICY="network"

if [ $# -lt 1 ]; then
    echo >&2 "Usage: $0 <lp|lp-routes|packet> [topology] [switch-radix] [traffic-type] [exp-seq] [num-networks] [transport-protocol] [routing-protocol] [k-shortest paths] [scheduling-mode]"
    exit 1
fi

MODE="$1"
case "$MODE" in
    "lp")
        SIM_MODE="lp-input"
        LINK_SPEED=10    # Gbps
        ;;
    "lp-routes")
        SIM_MODE="lp-routes"
        LINK_SPEED=10   # Gbps
        BUILD_SUBNAME+="$ROUTING_PROTOCOL."
        BUILD_SUBNAME+="$TRANSPORT_TYPE."
        BUILD_SUBNAME+="$SCHEDULING_MODE-plane."
        BUILD_SUBNAME+="K=$K_SHORTEST_PATHS."
        [ "$ROUTING_PROTOCOL" == "llskr" ] && BUILD_SUBNAME+="$SPREADING_POLICY-spread."
        ;;
    "packet")
        SIM_MODE="fluid"
        LINK_SPEED=10000 # Mbps
        BUILD_SUBNAME+="$ROUTING_PROTOCOL."
        BUILD_SUBNAME+="$TRANSPORT_TYPE."
        BUILD_SUBNAME+="$SCHEDULING_MODE-plane."
        BUILD_SUBNAME+="K=$K_SHORTEST_PATHS."
        [ "$ROUTING_PROTOCOL" == "llskr" ] && BUILD_SUBNAME+="$SPREADING_POLICY-spread."
        ;;
    *)
        echo >&2 "Unsupported mode \"$MODE\""
        exit 1
        ;;
esac

# Run configuration
BUILD_NAME="$MODE.k=$SWITCH_RADIX.$EXP_SEQ.$TRAFFIC_TYPE.$BUILD_SUBNAME$NUM_NETWORKS""net"
GRAPH_GROUP="$TOPOLOGY.k=$SWITCH_RADIX.$EXP_SEQ"

batch_mode=true
SCRIPT_DIR="$(pwd)"
BASE_DIR="$(realpath ../build/$BUILD_NAME)"
EXE_NAME="pnet/pnet_sim"
#BUILD_OUTPUT="CMakeFiles CMakeCache.txt cmake_install.cmake Makefile $EXE_NAME"
BUILD_OUTPUT="$EXE_NAME"
DEFAULT_BUILD_NAME="release"
DEFAULT_BUILD_DIR="$(realpath ../build/$DEFAULT_BUILD_NAME)"

# Cluster/Graph configuration
TRAFFIC_MODE="bulk"
FLOW_SIZE=$(echo "100*1024" | bc)
GRAPH="file"

# Read graph info from file based on switch radix
row=$(grep -E "^$SWITCH_RADIX," $TOPOLOGY.size.csv)
NUM_SWITCHES=$(echo $row | cut -f 2 -d ',')
if [ "$INCLUDE_SERVERS" == "true" ]; then
    NUM_HOSTS=$(echo $row | cut -f 3 -d ',')
    CMAKE_FLAGS+=" -DWITH_SERVERS=ON"
else
    NUM_HOSTS=$NUM_SWITCHES
    CMAKE_FLAGS+=" -DWITH_SERVERS=OFF"
fi
NUM_LINKS=$(echo $row | cut -f 4 -d ',')
NUM_LINKS=$( echo "$NUM_LINKS * 2" | bc)    # uni-directional intra-network links
if [ "$FIRST_FIT" == "true" ]; then
    CMAKE_FLAGS+=" -DWITH_FIRST_FIT=ON"
else
    CMAKE_FLAGS+=" -DWITH_FIRST_FIT=OFF"
fi
if [ "$LP_FAIR_SHARING" == "true" ]; then
    CMAKE_FLAGS+=" -DWITH_LP_FAIR_SHARING=ON"
else
    CMAKE_FLAGS+=" -DWITH_LP_FAIR_SHARING=OFF"
fi

exp_id=${EXP_SEQ:3} # Remove first three characters "exp"
TRAFFIC_MATRIX="tm.dat"
if [ "$TRAFFIC_MODE" == "bulk" ] && [ "$TRAFFIC_TYPE" == "permutation" ]; then
    TRAFFIC_TYPE="file"
    TRAFFIC_MATRIX="../../traffic_matrix/permutation.h=$NUM_HOSTS/tm$exp_id"
    if ! [ -f "$DEFAULT_BUILD_DIR/$TRAFFIC_MATRIX" ]; then
        TRAFFIC_TYPE="permutation"
        TRAFFIC_MATRIX="tm.dat"
    fi
fi

GUROBI_OPTIONS="ResultFile=gurobi.sol Method=2 CrossOver=0 BarHomogeneous=1"
GUROBI_OPTIONS+=" Threads=8"
: '
case "$NUM_NETWORKS" in
    "1")
        GUROBI_OPTIONS+=" Threads=1"
        ;;
    "2")
        GUROBI_OPTIONS+=" Threads=2"
        ;;
    "4")
        GUROBI_OPTIONS+=" Threads=4"
        ;;
    "8")
        # Use the remaining available threads
        # GUROBI_OPTIONS+=" Threads=1"
        ;;
esac
'

fast_build()
{
    if ! [ -d "$DEFAULT_BUILD_DIR" ]; then
        echo >&2 "Default build directory doesn't exist. Exiting ..."
        exit 2
    fi

    CWD=$(pwd)
    cd $DEFAULT_BUILD_DIR
    cmake . $CMAKE_FLAGS
    make -j $(nproc)
#    cp -r $BUILD_OUTPUT $CWD
    cd $CWD
}

build()
{
    echo >&2 "Using build \"$BUILD_NAME\"..."
    echo >&2
    mkdir -p "$BASE_DIR" && cd $BASE_DIR

    fast_build
    if ! [ -x "$DEFAULT_BUILD_DIR/$EXE_NAME" ]; then
        echo >&2 "Build failed. Aborting..."
        exit 2
    fi
    rm -rf run*
}

get_common_args()
{
    args="-m $SIM_MODE -t $TRAFFIC_TYPE --traffic_mode $TRAFFIC_MODE --traffic_matrix $TRAFFIC_MATRIX -k $SWITCH_RADIX -h $NUM_HOSTS -N $NUM_SWITCHES -l $NUM_LINKS"
    if [ "$TOPOLOGY" == "fat-tree" ]; then
        args+=" -g fat-tree"
    else
        args+=" -g $GRAPH"
    fi
    if [ "$MODE" == "packet" ]; then
        [ "$TRAFFIC_MODE" == "short" ] && args+=" --flow_size $FLOW_SIZE"
    fi
    if [ "$MODE" == "packet" ] ||  [ "$MODE" == "lp-routes" ]; then
        args+=" --transport $TRANSPORT_TYPE"
        args+=" --routing_protocol $ROUTING_PROTOCOL -K $K_SHORTEST_PATHS --scheduling_mode $SCHEDULING_MODE"
        [ "$ROUTING_PROTOCOL" == "llskr" ] && args+=" --th_w $TH_W --th_s $TH_S --spreading_policy $SPREADING_POLICY"
    fi
    echo $args
}

run_networks()
{
    local index=$1
    local network=${2:-"parallel"}  # single vs parallel
    local runid="run$index.$network"
    cd "$BASE_DIR"
    mkdir $runid
    cd $runid
    local args=$(get_common_args)
    if [[ "$network" == "parallel" ]]; then
        args+=" -s $LINK_SPEED"
    else    # single higher throughput network
        effective_link_speed=$(echo "$LINK_SPEED * $NUM_NETWORKS" | bc)
        args+=" -s $effective_link_speed"
    fi
    if [[ "$network" == "parallel" ]]; then
        for (( i = 1; i <= $NUM_NETWORKS; i += 1 )); do
            if [ "$index" == "all" ]; then
                [ "$TOPOLOGY" == "fat-tree" ] && args+=" --adjacency_list fat-tree.txt" || args+=" --adjacency_list ../../graphs/$GRAPH_GROUP/$TOPOLOGY$i.txt"
            else
                [ "$TOPOLOGY" == "fat-tree" ] && args+=" --adjacency_list fat-tree.txt" || args+=" --adjacency_list ../../graphs/$GRAPH_GROUP/$TOPOLOGY$index.txt"
            fi
        done
    else
        [ "$TOPOLOGY" == "fat-tree" ] && args+=" --adjacency_list fat-tree.txt" || args+=" --adjacency_list ../../graphs/$GRAPH_GROUP/$TOPOLOGY$index.txt"
    fi
    if [ "$MODE" == "lp" ]; then
        args+=" --lp_input $runid.lp"
    elif [ "$MODE" == "lp-routes" ]; then
        args+=" --lp_input $runid.lp --shortest_path shortest-path.$index.txt --flow_path flow-paths.$index.txt"
    elif [ "$MODE" == "packet" ]; then
        args+=" --shortest_path shortest-path.$index.txt --flow_path flow-paths.$index.txt"
    fi
    local cmd="$DEFAULT_BUILD_DIR/$EXE_NAME $args"
    # echo >&2 "$(pwd)> $cmd"
    echo "$cmd" > launch.cmd
    if [ "$MODE" == "lp" ] || [ "$MODE" == "lp-routes" ]; then
        local retval=1
        local run_num=1
        set +e  # gurobi might fail, we will re-generate LP and re-try
        # set -x
        while [[ $retval -ne 0 ]]; do
            echo "$runid: try #$run_num ..."
            /bin/rm -rf gurobi.log gurobi.sol
            { eval "time run_numa $cmd" > "$runid.log" 2> "$runid.err"; } 2>> "$runid.err"
            [ $? -ne 0 ] && { echo >&2 "Generate LP failed. Aborting ..."; exit 1; }
            { time run_numa gurobi_cl $GUROBI_OPTIONS $runid.lp > /dev/null; } 2>> gurobi.err
            grep "Optimal objective" gurobi.log > /dev/null
            retval=$?
            run_num=$((run_num + 1))
            if [ "$run_num" -gt 100 ]; then
                echo >&2 "Too many failed gurobi runs. Aborting ..."
                exit 1
            fi
            [ "$batch_mode" == true ] && rm flow-paths.*.txt logout.dat run*.lp shortest-path.*.txt run*.err
        done
    else
        { eval "time run_numa $cmd" > "$runid.log" 2> "$runid.err"; } 2>> "$runid.err"
    fi
}

get_stats()
{
    cd "$BASE_DIR"
    if [ "$MODE" == "lp" ]; then
        grep "Optimal objective" run*/gurobi.log > gurobi.stats.txt
    elif [ "$MODE" == "lp-routes" ]; then
        echo > gurobi.stats.txt
        for d in run*; do
            tput=$(cat $d/gurobi.sol| grep '^f_' | awk '{sum += $2} END {print sum}')
            echo "$d: total throughput $tput" >> gurobi.stats.txt
        done
    fi
}

main()
{
    build

    echo >&2
    echo >&2 "Launching jobs ..."
    echo >&2

    if [ "$MODE" == "packet" ]; then
        for (( index = 1; index <= NUM_NETWORKS; index += 1 ))
        do
            run_networks $index "single" &
            run_networks $index "parallel" &
        done
        run_networks "all" &
    elif [ "$MODE" == "lp" ]; then
        # LP mode: 1 network -> run all single cases;
        #          otherwise run parallel case
        if [ $NUM_NETWORKS -eq 1 ]; then
            for (( index = 1; index <= MAX_NETWORKS; index += 1 ))
            do
                run_networks $index "single" &
            done
        else
            run_networks "all" &
        fi
    else
        # LP routes mode
        for index in $(seq 1 $NUM_NETWORKS)
        do
            run_networks $index "single" &
            run_networks $index "parallel"  &
        done
        run_networks "all" "parallel" &
    fi

    sleep 1
    echo >&2
    echo >&2 "Build: $BUILD_NAME"
    echo >&2 "Build directory: $BASE_DIR"
    echo >&2 "Waiting for all jobs to finish ..."
    wait
    # [ "$MODE" = "lp" -o "$MODE" = "lp-routes" ] && get_stats
    cd "$SCRIPT_DIR"
}

set -e
#set -x

if [ "$batch_mode" = true ]; then
    if [[ $# -lt 12 ]]; then
        echo "Not enough parameters in batch mode ..."
        exit 1
    else
        mkdir -p "$BASE_DIR"
        run_networks ${11} ${12}
    fi
else
    main
fi

