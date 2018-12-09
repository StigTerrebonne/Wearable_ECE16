# -*- coding: utf-8 -*-
"""
@author: Matthew
"""

# ==================== Imports ====================
import serial
import numpy as np
import scipy.signal as sig
from filtering4 import process_ir
import time
from sklearn.mixture import GaussianMixture as GM
from matplotlib import pyplot as plt


# ==================== Globals ====================
N = 128                                      # number of samples plotted
NS = 16                                      # number of samples to read per iteration
NW = 32                                      #number to use to get past steps in welch
sample_count = 0                             # current sample count
steps = 0 
past_steps = 0
idle_counter = 0   #keeps track of how ong its been since the last step
f_samp = 25
f_hi = 0.1
f_lo = 7
order = 5
filt_coeffs = np.zeros((4, order+1))
filt_ICs = np.zeros ((4,2, order))
times, gyx, gyy, gyz, IR_Data = np.zeros((5, N))                # data vectors
filtered_vals = np.zeros ((4, N))
 
serial_port = 'COM6' # 'COM3'   # the serial port to use
serial_baud = 9600                              # baudrate


##======Globals for Calculating heartRate========== ##
beat_start = 0
beat_end = 0
beat_length = 0
beat_count = 0


buffer_size = 10
time_diffs = np.ones(buffer_size)
med_beat_length = 0
avg_time_diff = 1

reg_size = 10
length_info = np.zeros ((reg_size, 3)) #want length with start and stop index of last 5 vals
length_info = length_info.astype(int)

#================== functions ====================##

#============== this
def setup ():
    ser.write("AT+IMME1".encode('utf-8'))
    time.sleep(.5)
   
    ser.write("AT+ROLE1".encode('utf-8'))
    time.sleep(.5)
    ser.write("AT+RESET".encode('utf-8'))
    time.sleep(.5)
    
    ser.write("AT+CON3403DE02C3F7".encode('utf-8'))

        
    time.sleep(.5)
    
    return
    

def read_BLE( ser ):
	msg = ""
	if( ser.in_waiting > 0 ):
		msg = ser.readline( ser.in_waiting ).decode('utf-8')
		#print(msg)
	return msg

def connect ():  
    while (read_BLE(ser) != str(-1)):
        ser.write("AT+CON3403DE02C3F7".encode('utf-8'))
        print("Stuck in reconnenction")
        time.sleep(.5)
    print("im out")
    ser.write("CON".encode('utf-8'))
    time.sleep(0.5)
    return

# # ===== This function reads n_samples from serial and updates the global variables times/values =====
def grab_samples(n_samples) :
    global sample_count
    time, gyrox, gyroy, gyroz, irdata = np.zeros( (5, n_samples) )
    i = 0
    while i < n_samples :                      # while we still need samples, keep reading
        data = ""
        try:
            data = ser.readline().decode('utf-8').strip()
            #print (data)
            if (data == str(-2)): 
                connect()
              #  return time, gyrox, gyroy, gyroz
             
            t, gx, gy, gz, ir = data.split(' ')
            t = float(t) / 1000
            gx = float(gx)
            gy = float(gy)
            gz = float(gz)
            ir = float(ir)
            
            time[:-1] = time[1:]                  # shift and update our time/values arrays
            time[-1] = t
            
            gyrox[:-1] = gyrox[1:]
            gyrox[-1] = gx
            
            gyroy[:-1] = gyroy[1:]
            gyroy[-1] = gy
            
            gyroz[:-1] = gyroz[1:]
            gyroz[-1] = gz
            
            irdata[:-1] = irdata[1:]
            irdata[-1] = ir
            
            i += 1  #only increment when we get valid data

        except :                                # report error if we failed
            print('Invalid data: ', data)
        
       
    sample_count += n_samples
    return time, gyrox, gyroy, gyroz, irdata
    
# ==================== Grab new samples and plot ====================
def update_data():
    global times, gyx, gyy, gyz, filtered_vals, filt_ICs, steps, IR_Data, past_steps, idle_counter
    

    # shift samples left by 'NS'
    times[:N-NS] = times[NS:]
    gyx[:N-NS] = gyx[NS:]
    gyy[:N-NS] = gyy[NS:]
    gyz[:N-NS] = gyz[NS:]
    IR_Data[:N-NS] = IR_Data[NS:]
    
    times[N-NS:], gyx[N-NS:], gyy[N-NS:], gyz[N-NS:], IR_Data[N-NS:]= grab_samples(NS)
    
    filtered_vals[0][:N-NS] = filtered_vals[0][NS:]
    filtered_vals[1][:N-NS] = filtered_vals[1][NS:]
    filtered_vals[2][:N-NS] = filtered_vals[2][NS:]
    filtered_vals[3][:N-NS] = filtered_vals[3][NS:]
    
    filtered_vals[0][N-NS:], filt_ICs[0] = process_ir(gyx[N-NS:], filt_coeffs, filt_ICs[0], 0)
    filtered_vals[1][N-NS:], filt_ICs[1] = process_ir(gyy[N-NS:], filt_coeffs, filt_ICs[1], 0)
    filtered_vals[2][N-NS:], filt_ICs[2] = process_ir(gyz[N-NS:], filt_coeffs, filt_ICs[2], 0)
    filtered_vals[3][N-NS:], filt_ICs[3] = process_ir(IR_Data[N-NS:], filt_coeffs, filt_ICs[3], 0)
    # grab new samples
    samp4welch = NW / (times[-1] - times[-(NW+1)])
    freqs, pwr = sig.welch(filtered_vals[0][-(NW+1):-1] + filtered_vals[1][-(NW+1):-1] + filtered_vals[2][-(NW+1):-1], samp4welch, nperseg = NW) 
    idx = np.argmax(pwr)
    fmax = freqs [idx]  # frequency wehere the largest power occurred
    f_steps = fmax * 2 #there are two steps that are taken in one full cycle 
    
    if f_steps > 0.75 and f_steps < 4  and max(pwr) > 1e4:
        steps += f_steps * (times[-1] - times[-NS-1])
        
    if int(steps) == int(past_steps):
        idle_counter +=1
        if idle_counter == 10 and not int(steps) == 0:
            ser.write("b".encode('utf-8'))
    
    else: idle_counter = 0
    
    past_steps = steps
        
     
#    axes.set_xlim(times[0],times[N-1])
#    live_plot.set_data(times, filtered_vals[3])

    return steps

def build_filters():
    global f_samp, f_lo, f_hi, order, filt_coeffs, filt_ICs
    Wlow = f_lo / (f_samp / 2)
    Whi = f_hi / (f_samp / 2)
    b_low, a_low = sig.butter(order, Wlow, 'lowpass', analog=False,output='ba')
    b_hi, a_hi = sig.butter(order, Whi, 'highpass', analog=False, output='ba')
    filt_coeffs [0] = a_hi
    filt_coeffs [1] = b_hi
    filt_coeffs [2] = a_low
    filt_coeffs [3] = b_low
        
    zi_hi = sig.lfilter_zi(b_hi, a_hi)
    zi_low = sig.lfilter_zi(b_low, a_low)
        
    filt_ICs[0][0] = zi_hi
    filt_ICs[1][0] = zi_hi
    filt_ICs[2][0] = zi_hi
    filt_ICs[3][0] = zi_hi

    filt_ICs[0][1] = zi_low
    filt_ICs[1][1] = zi_low
    filt_ICs[2][1] = zi_low
    filt_ICs[3][1] = zi_low
    
    return

def calculate_hr():
    global time_diffs, beat_start
    for i in range (1, np.size (ir_labels)):
    
        if (ir_labels[i] != ir_labels[i-1]): 
            if (ir_labels[i] == 0): one_to_zero(i)
            if (ir_labels[i] == 1): beat_start = i
    
        if (i < buffer_size): continue

    i = 0    
    for i in range (1, np.size (ir_labels)):
        if (ir_labels[i] != ir_labels[i-1]): 
            if (ir_labels[i] == 1): 
                bpm = 60 / (times[i] - times[beat_start])
                if (bpm < 5 or bpm > 300 ):
                    beat_start = i
                    continue
                time_diffs = np.roll (time_diffs, 1)
                time_diffs [0] = times[i] - times[beat_start]
                beat_start = i
                
    
        if (i < buffer_size):
            continue

    HR = 60 / np.median(time_diffs)
    
    if HR < 40: ser.write("l".encode('utf-8'))
    elif HR> 120 : ser.write("h".encode('utf-8'))
    
    return HR

def one_to_zero(i):
    global beat_start, beat_end, beat_length,ir_labels, beat_count, med_beat_length, reg_size, length_info, time_diffs
   
    beat_count += 1
    beat_end = i
    beat_length = beat_end - beat_start
    info_idx = i % reg_size
    length_info [info_idx, 0] = beat_length
    length_info [info_idx, 1] = beat_start
    length_info [info_idx, 2] = beat_end
    
    if (beat_count < reg_size): return #allow register to fill up
    
    med_beat_length = np.median(length_info[:, 0])
    
    j = 0; 
    while (j < reg_size):
        
        percent_diff = abs (med_beat_length - length_info[j, 0])/ med_beat_length * 100
    
        if (percent_diff > 50): 
            ir_labels [length_info[j, 1]: length_info[j, 2]] = 0 #zero out start to end of false beat
        
        j += 1
    
def zero_to_one(i):
    global beat_start, time_diffs, times
    print("in zero to one")
    
    try: 
        bpm = 60 / (times[i] - times[beat_start])
        print (i)
        print(times[i])
        print (beat_start)
        print (times[beat_start])
        
        if (bpm < 10 or bpm > 250 ): return #dont add extreme values to the average
    
    except: return
    
    
    time_diffs = np.roll (time_diffs, 1)
    time_diffs [0] = times[i] - times[beat_start]
    beat_start = i
    
    return
    

# ==================== Main ====================
# Note: this assumes data is being sent by the Arduino in the format of "<time> <value>"
# You will need to modify it to suit your needs!
if (__name__ == "__main__") :
  
    # Open Serial
    with serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1) as ser:
        
        setup()
        
        build_filters()
          # ser.write(b'1')                         # tell Arduino to start sending data. NOTE!!: you should change this for your setup 
       
        times, gyx, gyy, gyz, IR_Data= grab_samples(N) 
 
        filtered_vals[0], filt_ICs[0] = process_ir(gyx, filt_coeffs, filt_ICs[0], 1)
        filtered_vals[1], filt_ICs[1] = process_ir(gyy, filt_coeffs, filt_ICs[1], 1)
        filtered_vals[2], filt_ICs[2] = process_ir(gyz, filt_coeffs, filt_ICs[2], 1)
        filtered_vals[3], filt_ICs[3] = process_ir(IR_Data, filt_coeffs, filt_ICs[3], 1)
        
        gmm = GM(n_components = 2)
        gmm_data = np.reshape(filtered_vals[3], (np.size(filtered_vals[3]), 1))
        gmm_fit = gmm.fit(  gmm_data  )
        ir_labels = gmm_fit.predict(  gmm_data  )
        
        HR = calculate_hr()
        print (HR)
      
        plt.plot(times, filtered_vals[3])
        plt.plot(times, ir_labels * 100)
        plt.show()
#        
#        fig, axes = plt.subplots()
#
#        live_plot = axes.plot(times, ir_labels, lw=2)[0]
#        
#        anim = animation.FuncAnimation(fig, update_data, interval=1000)
#        plt.show()
        m = 0
        while (True): 
            steps = update_data()
            gmm_data = np.reshape(filtered_vals[3], (np.size(filtered_vals[3]), 1))
            gmm_fit = gmm.fit(  gmm_data  )
            ir_labels = gmm_fit.predict(  gmm_data  )
            HR = calculate_hr()
            print (HR)
            if m % 10 == 0:
                finalString = str(steps) + "," + str(HR)
                ser.write(str(finalString).encode('utf-8'))
            m+=1