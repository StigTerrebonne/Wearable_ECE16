# -*- coding: utf-8 -*-
"""
@author: Matthew
"""

# ==================== Imports ====================
import serial
import numpy as np
import pyrebase
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
heart_counter = 0
f_samp = 25                                  #sampling frequency
f_hi = 0.1                                   # cutoff for highpass filter
f_lo = 7                                     #cutoff for lowpass filter
order = 5                                    #filter order
filt_coeffs = np.zeros((4, order+1))
filt_ICs = np.zeros ((4,2, order))
times, gyx, gyy, gyz, IR_Data = np.zeros((5, N))                # global data vectors
filtered_vals = np.zeros ((4, N))           #holds filtered gyroscope x, y, z, and IR
 
serial_port = 'COM6' # 'COM3'   # the serial port to use
serial_baud = 9600                              # baudrate


##======Globals for Calculating heartRate========== ##
beat_start = 0
beat_end = 0
beat_length = 0
beat_count = 0


buffer_size = 10                            #these are the time differences between the start
time_diffs = np.ones(buffer_size)           #and end of the beat
med_beat_length = 0
avg_time_diff = 1

reg_size = 10                               
length_info = np.zeros ((reg_size, 3)) #want length with start and stop index of last 10 vals
length_info = length_info.astype(int)  

##======Globals for Calculating Steps========== ##
steps = 0 
past_steps = 0
idle_counter = 0                             #keeps track of how long its been since the last step
low_step_freq = 0.75
high_step_freq = 4
power_thresh = 1e4

#================== Setups for PyreBase ====================##

config = {
  "apiKey": "AIzaSyBCuJvm-DPjvS6P9TQ-sXhs01g76e9aWto",
  "authDomain": "ece16-fall18.firebaseapp.com",
  "databaseURL": "https://ece16-fall18.firebaseio.com",
  "storageBucket": "ece16-fall18.appspot.com",
}
firebase = pyrebase.initialize_app(config)
db = firebase.database()
last_time = time.time()

def write_to_pyrebase(teamID, hr, steps):
    assert isinstance(teamID, str)
    assert isinstance(hr, int)
    assert isinstance(steps, int)
    
    global last_time
    current_time = time.time()
    if (current_time - last_time >= 0.5):
        last_time = current_time
        data = {"teamID": teamID, "hr": hr, "steps": steps, "timestamp": current_time}
        db.child("readings").push(data)
        
#============================ MY DEFINED FUNCTIONS ==========================##

#setup allows for the bluetooth modules to connect to eachother 
def setup ():
    ser.write("AT+IMME1".encode('utf-8'))
    time.sleep(.5)
   
    ser.write("AT+ROLE1".encode('utf-8'))
    time.sleep(.5)
    ser.write("AT+RESET".encode('utf-8'))
    time.sleep(.5)
    
    ser.write("AT+CON3403DE02C3F7".encode('utf-8')) #CON4006A09500D0

        
    time.sleep(.5)
    
    return
    
# connect() function uses this to read the BLE when trying to reconnect
def read_BLE( ser ):
	msg = ""
	if( ser.in_waiting > 0 ):
		msg = ser.readline( ser.in_waiting ).decode('utf-8')
		#print(msg)
	return msg

# connect() is trying to reconnect to the arduino after the button has been pressed
def connect ():  
    while (read_BLE(ser) != str(-1)):
        ser.write("AT+CON4006A09500D0".encode('utf-8'))
        print("Stuck in reconnenction")
        time.sleep(.5)
    print("im out")
    ser.write("CON".encode('utf-8'))
    time.sleep(0.5)
    return

# reads n_samples from serial and updates the global variables times/values
def grab_samples(n_samples) :
    global sample_count
    time, gyrox, gyroy, gyroz, irdata = np.zeros( (5, n_samples) ) #temp data arrays of n_samples
    i = 0
    while i < n_samples :                      # while we still need samples, keep reading
        data = ""
        try:
            data = ser.readline().decode('utf-8').strip()
            #print (data)
            if (data == str(-2)):   #arduino sends over a -2 when disconnecting
                connect()
              #  return time, gyrox, gyroy, gyroz
             
            t, gx, gy, gz, ir = data.split(' ') #temp data vals to be stored into temp array
            t = float(t) / 1000
            gx = float(gx)
            gy = float(gy)
            gz = float(gz)
            ir = float(ir)
            
            # shift and update our temp arrays with temp values for each read
            time[:-1] = time[1:]                  
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

        except :                                # report error if we failed, dont increment
            print('Invalid data: ', data)
        
       
    sample_count += n_samples
    return time, gyrox, gyroy, gyroz, irdata   #return temp arrays to be stored in main data array
    
# update_data() takes uses grab_samples() to get new samples and filters the new data
def update_data():
    global times, gyx, gyy, gyz, filtered_vals, filt_ICs, IR_Data #bring over main data arrays

    # shift samples of main array left by 'NS'
    times[:N-NS] = times[NS:]
    gyx[:N-NS] = gyx[NS:]
    gyy[:N-NS] = gyy[NS:]
    gyz[:N-NS] = gyz[NS:]
    IR_Data[:N-NS] = IR_Data[NS:]
    
    # get new data and save into the last bit of the main array
    times[N-NS:], gyx[N-NS:], gyy[N-NS:], gyz[N-NS:], IR_Data[N-NS:]= grab_samples(NS)
    
    filtered_vals[0][:N-NS] = filtered_vals[0][NS:]
    filtered_vals[1][:N-NS] = filtered_vals[1][NS:]
    filtered_vals[2][:N-NS] = filtered_vals[2][NS:]
    filtered_vals[3][:N-NS] = filtered_vals[3][NS:]
    
    filtered_vals[0][N-NS:], filt_ICs[0] = process_ir(gyx[N-NS:], filt_coeffs, filt_ICs[0], 0)
    filtered_vals[1][N-NS:], filt_ICs[1] = process_ir(gyy[N-NS:], filt_coeffs, filt_ICs[1], 0)
    filtered_vals[2][N-NS:], filt_ICs[2] = process_ir(gyz[N-NS:], filt_coeffs, filt_ICs[2], 0)
    filtered_vals[3][N-NS:], filt_ICs[3] = process_ir(IR_Data[N-NS:], filt_coeffs, filt_ICs[3], 0)

    return get_steps()

# get_steps() calculates steps based on the welch function and time differences
def get_steps():
    global steps, times, filtered_vals, idle_counter, past_steps, NW
    
    #freq4welch is the real frequency based off of the last NW samples and the times
    freq4welch = NW / (times[-1] - times[-(NW+1)])
    
    # use the summation of the gyroscope x, y, and z data for welch
    freqs, pwr = sig.welch(filtered_vals[0][-(NW+1):-1] + filtered_vals[1][-(NW+1):-1] + filtered_vals[2][-(NW+1):-1], freq4welch, nperseg = NW) 
    
    idx = np.argmax(pwr) #index where the maximum power occurs
    fmax = freqs [idx]  # frequency wehere the largest power occurred
    f_steps = fmax * 2 #there are two steps that are taken in one full cycle 
    
    #make sure frequeny and power in the correct range before adding to step total
    if f_steps > low_step_freq and f_steps < high_step_freq  and max(pwr) > power_thresh:
        steps += f_steps * (times[-1] - times[-NS-1])
       
    #keep track if the user hasn't moved in a while    
    if int(steps) == int(past_steps):
        idle_counter +=1
        if idle_counter > 50 and not int(steps) == 0:
            ser.write("b".encode('utf-8'))
            idle_counter = 0
    
    else: idle_counter = 0
    
    past_steps = steps
        
    return steps

# build_filters() sets up the highpass and lowpass filters needed for heartrate and steps
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

# calculate_hr() gets the heartrate by using the predicted labels, clearing our irregularities
# and then counts the time differences between when label goes from 0 to 1
def calculate_hr():
    global time_diffs, beat_start, heart_counter
    
    #loop through the labels once and clear out the irregularities 
    for i in range (1, np.size (ir_labels)):
    
        if (ir_labels[i] != ir_labels[i-1]): 
            if (ir_labels[i] == 0): one_to_zero(i) # when 1 to 0, use this function to get length
            if (ir_labels[i] == 1): beat_start = i #when 0 to 1, get the index when it starts
    
        if (i < buffer_size): continue             # we want buffer to fill up before performing calcs

    
    #loop through a second time and note the 0 to 1 changes to calculate rate
    i = 0    
    for i in range (1, np.size (ir_labels)):
        if (ir_labels[i] != ir_labels[i-1]): #if past value doesnt equal the current val, there was a value change
            if (ir_labels[i] == 1):          # notes the 0 to 1 changes
                
                bpm = 60 / (times[i] - times[beat_start]) #this is just used to help clear wrong values
                if (bpm < 5 or bpm > 300 ):               #dont want extreme vals to be added
                    beat_start = i
                    continue
                time_diffs = np.roll (time_diffs, 1)        #time differences between beat starts
                time_diffs [0] = times[i] - times[beat_start]
                beat_start = i
                
    
        if (i < buffer_size):                       #want buffer to fill before returning value
            continue

    HR = 60 / np.median(time_diffs)                 #change to beats per minute based off average time differences
    
    #send warnings if bad values over a long period of time
    if HR < 20: 
        heart_counter += 1
        if heart_counter > 50:
            ser.write("l".encode('utf-8'))
            heart_counter = 0
    elif HR> 170 : 
        heart_counter += 1
        if heart_counter > 50:
            ser.write("h".encode('utf-8'))
    else: heart_counter = 0
    
    return HR

#this takes the index of the value change from a 1 to zero and determins the length of the beat
def one_to_zero(i):
    global beat_start, beat_end, beat_length,ir_labels, beat_count, med_beat_length, reg_size, length_info, time_diffs
   
    beat_count += 1 
    beat_end = i                            #beat ended at this index that was sent
    beat_length = beat_end - beat_start     #length is the difference between the end and the start
    info_idx = i % reg_size                 # want to save into a register without overwriting 
    length_info [info_idx, 0] = beat_length # length info holds all 3 of these vals
    length_info [info_idx, 1] = beat_start
    length_info [info_idx, 2] = beat_end
    
    if (beat_count < reg_size): return #allow register to fill up
    
    med_beat_length = np.median(length_info[:, 0])  #using the median length to get rid of extreme values
    
    #compare each length in the register with the median value and clear out extremes
    j = 0; 
    while (j < reg_size):
        
        percent_diff = abs (med_beat_length - length_info[j, 0])/ med_beat_length * 100
    
    #zero out labels where the extreme differences took place
        if (percent_diff > 50): 
            ir_labels [length_info[j, 1]: length_info[j, 2]] = 0 #zero out start to end of false beat
        
        j += 1
    

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
 
    #get the initial filtered values from the beginning N values
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
        plt.plot(times, ir_labels )
        plt.show()

        m = 0
        while (True): 
            steps = update_data()
            gmm_data = np.reshape(filtered_vals[3], (np.size(filtered_vals[3]), 1))
            gmm_fit = gmm.fit(  gmm_data  )
            ir_labels = gmm_fit.predict(  gmm_data  )
            HR = calculate_hr()
            
            #only print every 10 iterations 
            if m % 10 == 0:
                finalString = str(steps) + "," + str(HR)
                ser.write(str(finalString).encode('utf-8'))
                #write_to_pyrebase("CarneAsadaFries", HR, steps)
            m+=1