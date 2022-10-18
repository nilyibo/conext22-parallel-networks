#!/bin/bash

function run_numa()
{
    command -v numactl >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo >&2 "[WARN] numactl not installed. Running directly ..."
        eval "$@"
    else
        set -e
        nodes=$(numactl -H | grep "available: " | awk '{print $2}')
        node=$(bc <<< "$RANDOM % $nodes")
        numactl --cpunodebind $node --localalloc "$@"
    fi
}

function isem () {
    sem --id $(hostname -s).$(tty) -j $(nproc) "$@"
}

function isemwait () {
    sem --id $(hostname -s).$(tty) --wait
}
