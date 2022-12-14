# conext22-parallel-networks
Code artifact for CoNEXT'22 paper "Scaling beyond packet switch limits with multiple dataplanes"

You can find the most up-to-date version on [GitHub](https://github.com/nilyibo/conext22-parallel-networks) and [Zenodo](https://doi.org/10.5281/zenodo.7196266).

The artifact for this paper consists of
1. a C++ packet simulator on top of htsim from the NDP paper
2. a set of scripts to run the LP solver Gurobi, and
3. the plotting scripts in the form of Jupyter notebooks to reproduce the figures in the evaluation sections.

We'll describe the setup and run procedure next.

## Setup

### Simulator

The C++ simulator is based on [htsim packet simulator](https://github.com/kellianhunt/htsim/tree/master/sim), which is included in this repo.

You can find the simulator code in [sim](./sim) directory as well as the [setup script](./sim/scripts/setup.sh) that pulls/installs its dependencies and builds the simulator.

### Gurobi LP Solver

We use a commercial LP solver [Gurobi](https://www.gurobi.com/) that offers free academic license.
Please follow the [instructions](https://www.gurobi.com/academia/academic-program-and-licenses/) to obtain a license and set up gurobi on your machine.
When set up properly, you should be able to run `gurobi.sh` and get an interactive prompt without license errors.

We used gurobi version 9.0.1.
```
$ gurobi_cl --version
Gurobi Optimizer version 9.0.1 build v9.0.1rc0 (linux64)
Copyright (c) 2020, Gurobi Optimization, LLC
```

### Plotting script / Jupyter notebook

Plotting script can be found in [plots](./plots) directory, organized by evaluation subsections.
It runs on standard [Jupyter `notebook`](https://jupyter.org/install) + `numpy` + `matplotlib`, so please install that with your package manager (`pip`/`conda`/...).

## How to run the simulator?

The simulator should be built as a binary located at [sim/build/release/pnet/pnet_sim](./sim/build/release/pnet/pnet_sim).
For each experiment, we include the scripts to run the simulator with appropriate arguments, e.g. mode (latency vs throughput vs LP solver input vs generate traffic matrix), network topology, traffic matrix, ... You can find them [here](./reproduce).

The scripts will also perform necessary pre-processing and put the result under `data` directory in the repo root. They will also delete any temporary data to avoid too much disk usage.

Currently, this only covers the [bulk traffic](./reproduce/micro.bulk.sh) part, but we plan to add the rest later.

## How to interpret simulator output and/or reproduce the plots?

Simulator output will be parsed by the included script and simpler text/CSV format results (e.g. flow completion times, throughput, ...) will be saved in the `data` directory, which are then processed by the Jupyter notebooks in [plots](./plots) for plotting.
