#!/usr/bin/env python
# encoding: utf-8
"""
plot_eeprom.py

Created by Jonny Dyer on 2018-01-01.
Copyright (c) 2018 Jonny Dyer Inc.. All rights reserved.
"""

import sys
import os
import numpy as np
from matplotlib import pyplot as plt
from struct import unpack
from bitstring import BitStream

def main():
    with open("eeprom.bin", "rb") as f:
        bytes = bytearray(f.read())
            
    bytes = bytearray([b for b in reversed(bytes)])
    bs = BitStream(bytes)
    size = len(bytes) / 4
    curr = np.zeros([size], dtype=np.float64)
    volt = np.zeros([size], dtype=np.float64)
    temp = np.zeros([size], dtype=np.float64)
    for i in xrange(size):
         curr[i] = bs.read(12).uint * (3.35693e-3 * 4)
         volt[i] = 1.1 * 1023.0 / bs.read(10).uint
         temp[i] = bs.read(10).uint
    
    print curr, volt, temp
    plt.figure()
    plt.plot(curr)
    
    plt.figure()
    plt.plot(volt)
    
    plt.figure()
    plt.plot(temp)
    plt.show()

if __name__ == '__main__':
	main()

