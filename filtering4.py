# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 11:53:59 2018

@author: Matthew
"""


"""
This function processes the data in 3 steps
    Step 1: Normalize from ADC value (0-1023) to voltage
    Step 2: Apply LPF filter
    Step 3: Apply HPF filter

INPUT:
    data:          a numpy array, with dimension (N,), contains data to be processed

    filter_coeffs: a numpy array, with dimensions (4, M+1), contains filter coefficients
                   row 0:  HPF a coefficients
                   row 1:  HPF b coefficients
                   row 2:  LPF a coefficients
                   row 3:  LPF b coefficients        

    filter_ICs:    a numpy array, with dimension (2, M), contains initial conditions
                   row 0: HPF initial conditions
                   row 1: LPF initial conditions

    Note, M is the filter order number.
    
OUTPUT: 
    proc_data:     a numpy array, with dimension (N,), contains processed dat
    filter_ICs:    a numpy array, with dimension (2, M), contains new filter initial conditions
"""

# ==================== Imports ====================
import numpy as np
import scipy.signal as sig

# ==================== Function ====================
def process_ir(data, filter_coeffs, filter_ICs, n):
    if (n ==1):
       
        data, filter_ICs[1] = sig.lfilter(filter_coeffs[3], filter_coeffs[2], data, zi = filter_ICs[1] * data[0])
        data, filter_ICs[0] = sig.lfilter(filter_coeffs[1], filter_coeffs[0], data, zi=filter_ICs[0] * data[0])

    else :
       
        data, filter_ICs[1] = sig.lfilter(filter_coeffs[3], filter_coeffs[2], data, zi = filter_ICs[1])
        data, filter_ICs[0] = sig.lfilter(filter_coeffs[1], filter_coeffs[0], data, zi=filter_ICs[0])
        
    return data, filter_ICs

