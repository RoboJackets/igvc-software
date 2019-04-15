import numpy as np
from math import floor

# Opens configuration file for locations of training related files.
# Paths are delimited by '='.
def read_data_cfg(datacfg):
    options = dict()
    with open(datacfg, 'r') as fp:
        lines = fp.readlines()

    for line in lines:
        line = line.strip()
        if line == '' or line[0] == '#':
            continue
        key,value = line.split('=')
        key = key.strip()
        value = value.strip()
        options[key] = value
    return options
