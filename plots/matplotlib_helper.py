#!/usr/bin/env python

import sys
import matplotlib
import numpy as np
from packaging import version

import matplotlib.pyplot as plt

def get_next_color():
    ax = plt.gca()
    if version.parse(matplotlib.__version__) < version.parse('1.5'):
        ax_color_cycle = ax._get_lines.color_cycle
    else:
        ax_color_cycle = ax._get_lines.prop_cycler
    if sys.version_info.major == 2:
        color = ax_color_cycle.next()
    else:
        color = next(ax_color_cycle)
    if version.parse(matplotlib.__version__) >= version.parse('1.5'):
        color = color['color']
    return color

def get_linestyle(index):
    styles = [
        'solid',
        'dotted',
        'dashed',
        'dashdot',
        (0, (3, 5, 1, 5, 1, 5)), # 'dashdotdotted'
        (0, (5, 10)), # 'loosely dashed'
        (0, (3, 1, 1, 1)), # 'densely dashdotted'
        (0, (3, 10, 1, 10, 1, 10)), # 'loosely dashdotdotted'
        (0, (3, 1, 1, 1, 1, 1)), #'densely dashdotdotted'

    ]
    return styles[index % len(styles)]

def get_marker(index):
    markers = [
        '.',
        'o',
        's',
        'd',
        'v',
        '^',
    ]
    return markers[index % len(markers)]

def get_pattern(index):
    patterns = [ " ", "\\" , "/" , "|" , "+" , "-", ".", "*","x", "o", "O" ]
    return patterns[index % len(patterns)]

#  Returns tuple of handles, labels for axis ax, after reordering them to conform to the label order `order`, and if unique is True, after removing entries with duplicate labels.
def reorder_legend(ax=None,order=None,unique=False, **kwargs):
    if ax is None: ax=plt.gca()
    handles, labels = ax.get_legend_handles_labels()
    labels, handles = zip(*sorted(zip(labels, handles), key=lambda t: t[0])) # sort both labels and handles by labels
    if order is not None: # Sort according to a given list (not necessarily complete)
        keys=dict(zip(order,range(len(order))))
        labels, handles = zip(*sorted(zip(labels, handles), key=lambda t,keys=keys: keys.get(t[0],np.inf)))
    if unique:  labels, handles= zip(*unique_everseen(zip(labels,handles), key = labels)) # Keep only the first of each handle
    ax.legend(handles, labels, **kwargs)
    return(handles, labels)

def autolabel(ax, rects, xpos='center'):
    """
    Attach a text label above each bar in *rects*, displaying its height.

    *xpos* indicates which side to place the text w.r.t. the center of
    the bar. It can be one of the following {'center', 'right', 'left'}.
    """

    ha = {'center': 'center', 'right': 'left', 'left': 'right'}
    offset = {'center': 0, 'right': 1, 'left': -1}

    for rect in rects:
        height = rect.get_height()
        ax.annotate('{:.2f}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(offset[xpos]*0, 3),  # use 3 points offset
                    textcoords="offset points",  # in both directions
                    ha=ha[xpos], va='bottom')

def unique_everseen(seq, key=None):
    seen = set()
    seen_add = seen.add
    return [x for x,k in zip(seq,key) if not (k in seen or seen_add(k))]
