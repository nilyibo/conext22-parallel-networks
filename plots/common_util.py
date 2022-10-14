#!/usr/bin/env python3

import re, sys
import numpy as np
from network_topology import *
from matplotlib_helper import *
import matplotlib.pyplot as plt
from matplotlib.ticker import *

save_figures = False

plt.rcParams.update({'font.size': 12})

def parse_fct(filename):
    # Example: '4936 (1048576B) 0.004762s'
    regex = re.compile(r'(\d+) \((\d+)B\) ([\d.e-]+)s')
    data = {}
    with open(filename) as f:
        for line in f:
            m = regex.match(line)
            if not m:
                raise ValueError('Cannot parse line "%s"' % line)
            flow_id = int(m.group(1))
            flow_size = int(m.group(2))
            flow_fct = float(m.group(3))
            data[flow_id] = (flow_size, flow_fct)
    return data


def parse_log(filename):
    # Example: 'Flow ID 4120 (102400B) finished after 8.68648e-05s'
    regex = re.compile(r'Flow ID (\d+) \((\d+)B\) finished after ([\d.e-]+)s')
    data = {}
    line_count = 0
    with open(filename) as f:
        for line in f:
            line_count += 1
            line = line.strip()
            if not line.startswith('Flow ID '):
                continue
            m = regex.match(line)
            if not m:
                raise ValueError('Cannot parse line "%s"' % line)
            flow_id = int(m.group(1))
            flow_size = int(m.group(2))
            flow_fct = float(m.group(3))
            data[flow_id] = (flow_size, flow_fct)
    # eprint('read %d lines from file "%s"' % (line_count, filename))
    return data

def parse_ids_from_log(filename):
    d = {}
    line_count = 0
    with open(filename, 'rb') as f:
        for line in f:
            line_count += 1
            line = line.strip().decode('utf-8')
            if line.startswith('#'):
                # start of binary data
                break
            assert line.startswith(':')
            line = line.lstrip(': ')
            arr = line.split('=')
            assert len(arr) == 2
            name = arr[0]
            id = int(arr[1])
            d[id] = name
    # eprint('read %d lines from file "%s"' % (line_count, filename))
    return d

# Use id map that has id -> either tcp_src(a->b) or tcp_src(a->b)-response and return a map from request to response
def get_request_to_response_id_map(id_map):
    name_regex = re.compile(r'([\w_.]+)\((\d+)->(\d+)\)(.repeat\d+)?')
    request_id_map = {}
    response_id_map = {}
    for id, name in id_map.items():
        if 'tcp' not in name:
            continue
        suffix = '-response'
        if name.endswith(suffix):
#             eprint(name)
            name = name[:-len(suffix)]
            m = name_regex.match(name)
            if not m:
                raise ValueError('Unable to parse name %s' % name)
            name_prefix = m.group(1)
            name_src = m.group(3)  # response flow src/dst is reversed
            name_dst = m.group(2)
            name_suffix = m.group(4) or ''
            name = '%s(%s->%s)%s' % (name_prefix, name_src, name_dst, name_suffix)
            response_id_map[id] = name
        else:
            request_id_map[id] = name
#     eprint(sorted(request_id_map.values()))
#     eprint(sorted(response_id_map.values()))
    assert set(request_id_map.values()) == set(response_id_map.values())
    return request_id_map, response_id_map
#     request_to_response_id_map = {}
#     for name, id in request_name_map:
#         request_id = id
#         response_id = response_name_map[name]
#         request_to_response_id_map[request_id] = response_id
#     return request_to_response_id_map

def eprint(*args):
    print(*args, file=sys.stderr)

def eprint_array(array, label='array'):
    eprint('%s\t: %s' % (label, ', '.join('%0.3g' % item for item in array)))

def plot_cdf_array(array, label, include_count = False, index=0, color=None, ax=None):
    x = sorted(array)
    y = np.linspace(0., 1., len(array) + 1)[1:]
    if include_count:
        label += ' (%d)' % len(array)
    if color is None:
        color = get_next_color()
    if ax is None:
        plt.plot(x, y, label=label, color=color, linestyle=get_linestyle(index))
    else:
        ax.plot(x, y, label=label, color=color, linestyle=get_linestyle(index))

def get_fct_array(data):
    array = []
    for flow_id in data:
        fct_value = data[flow_id][1]
        array.append(fct_value)
    return array

def get_stats(m):
    average_v = [np.average(array) for _, array in m.items()]
    median_v = [array[len(array) // 2] for _, array in m.items()]
    p90_v = [array[int(len(array) * 0.90)] for _, array in m.items()]
    p99_v = [array[int(len(array) * 0.99)] for _, array in m.items()]
    max_v = [array[-1] for _, array in m.items()]
    min_v = [array[0] for _, array in m.items()]
    return {
        'average': average_v,
        'median': median_v,
        'percent90': p90_v,
        'percent99': p99_v,
        'max': max_v,
        'min': min_v
    }

def get_exp_name(dir_name):
    map_names = {
        'single': 'Serial',
        'homogeneous': 'Parallel homogeneous',
        'heterogeneous': 'Parallel heterogeneous',
        'large': 'Serial'
    }
    if dir_name in map_names:
        return map_names[dir_name]
    else:
        raise ValueError('Unrecognized name %s' % dir_name)

m_suffix_ratio = {
    None: 1,
    'kB': pow(10, 3),
    'MB': pow(10, 6),
    'GB': pow(10, 9),
}

def parse_size(s):
    regex = re.compile(r'([\d.]+)(kB|MB|GB)?')
    m = regex.match(s)
    if not m:
        raise ValueError('Cannot parse size %s' % s)
    base = float(m.group(1))
    ratio = m_suffix_ratio[m.group(2)]
    return int(base * ratio)

def humanize_size(size_in_bytes):
    if size_in_bytes < m_suffix_ratio['kB']:
        return "%dB" % size_in_bytes
    elif size_in_bytes < m_suffix_ratio['MB']:
        return "%dkB" % int(size_in_bytes/m_suffix_ratio['kB'])
    elif size_in_bytes < m_suffix_ratio['GB']:
        return "%dMB" % int(size_in_bytes/m_suffix_ratio['MB'])
    else:
        return "%dGB" % int(size_in_bytes/m_suffix_ratio['GB'])

def plot_errorbar(x, y, yerr, label, color='b', linestyle='solid', marker='.', fillstyle=None):
    eprint('[', label, ']')
    eprint_array(x, 'x')
    eprint_array(y, 'y')
    eprint_array(yerr, 'yerr')
    ax = plt.gca()
    ax.errorbar(x, y, yerr=yerr, label=label, color=color, linestyle=linestyle, marker=marker, fillstyle=fillstyle)

def get_exp_name(dir_name):
    map_names = {
        'single': 'Serial low-bw',
        'homogeneous': 'Parallel homogeneous',
        'heterogeneous': 'Parallel heterogeneous',
        'large': 'Serial high-bw'
    }
    if dir_name in map_names:
        return map_names[dir_name]
    else:
        raise ValueError('Unrecognized name %s' % dir_name)

MARKER_INDEX = {
    'single': 1,
    'large': 4,
    'homogeneous': 2,
    'heterogeneous': 3,
}


