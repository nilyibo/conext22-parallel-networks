#!/usr/bin/env python2.7

import re, sys

class NetworkTopology:
    RandomGraph = "Random Graph"
    FatTree = "Fat-Tree"
    Hypercube = "Hypercube"
    _2DTorus = "2D-Torus"
    FlattenedButterfly = "Flattened-BF"
    Dragonfly = "Dragonfly"

def get_topology(name):
    # print >> sys.stderr, name
    if re.search(r'random', name, re.IGNORECASE):
        return NetworkTopology.RandomGraph
    elif re.search(r'fat.*tree', name, re.IGNORECASE):
        return NetworkTopology.FatTree
    elif re.search(r'hyper.*cube', name, re.IGNORECASE):
        return NetworkTopology.Hypercube
    elif re.search(r'2d.*torus', name, re.IGNORECASE):
        return NetworkTopology._2DTorus
    elif re.search(r'flattened.*(BF|butterfly)', name, re.IGNORECASE):
        return NetworkTopology.FlattenedButterfly
    elif re.search(r'dragonfly', name, re.IGNORECASE):
        return NetworkTopology.Dragonfly
    else:
        return None

def get_topology_color_label(topology):
    if topology is NetworkTopology.RandomGraph:
        index = 0
    elif topology is NetworkTopology.FatTree:
        index = 1
    elif topology is NetworkTopology.Hypercube:
        index = 2
    elif topology is NetworkTopology._2DTorus:
        index = 3
    elif topology is NetworkTopology.FlattenedButterfly:
        index = 4
    elif topology is NetworkTopology.Dragonfly:
        index = 5
    else:
        return 'b'
    return 'C' + str(index - 1)

def get_topology_marker(topology):
    # See possible options here: https://matplotlib.org/3.1.1/api/markers_api.html
    return '*'
    if topology is NetworkTopology.RandomGraph:
        return '.'
    elif topology is NetworkTopology.FatTree:
        return 'v'
    elif topology is NetworkTopology.Hypercube:
        return '^'
    elif topology is NetworkTopology._2DTorus:
        return '<'
    elif topology is NetworkTopology.FlattenedButterfly:
        return '>'
    elif topology is NetworkTopology.Dragonfly:
        return '1'
    else:
        return ''

def get_experiment_name(name):
    d = {
        'single': 'Serial Low Bandwidth',
        'homogeneous': 'Parallel Homogeneous',
        'heterogeneous': 'Parallel Heterogeneous',
        'large': 'Serial',
        'high-radix': 'High Radix'
    }
    if name in d:
        return d[name]
    eprint('Unsupported experiment name "%s"' % name)
    return name
