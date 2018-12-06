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
#from matplotlib import pyplot as plt
#from matplotlib import animation

# ==================== Globals ====================
N = 200                                         # number of samples plotted
NS = 10                                       # number of samples to read per iteration
sample_count = 0                                # current sample count
steps = 0 
f_samp = 25
f_hi = 0.1
f_lo = 7
order = 5
filt_coeffs = np.zeros((4, order+1))
filt_ICs = np.zeros ((7,2, order))
times, accx, accy, accz, gyx, gyy, gyz, IR_Data = np.zeros((8, N))                # data vectors
filtered_vals = np.zeros ((7, N))
 
serial_port = 'COM6' # 'COM3'   # the serial port to use
serial_baud = 9600                              # baudrate

step_start = step_end = step_length= step_count = 0
reg_size = 10
length_info = np.zeros ((reg_size, 3)) #start, end, and length of past 3 steps
length_info = length_info.astype(int)
med_beat_length = 0
avg_time_diff = 1

HR=0

def setup ():
    ser.write("AT+IMME1".encode('utf-8'))
    time.sleep(.5)
   
    ser.write("AT+ROLE1".encode('utf-8'))
    time.sleep(.5)
    ser.write("AT+RESET".encode('utf-8'))
    time.sleep(.5)
    
    ser.write("AT+CON3403DE02C3F7".encode('utf-8'))

        
    time.sleep(.5)
    

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
    global sample_count, times, accx, accy, accz, gyx, gyy, gyz, IR_Data
    time, accelx, accely, accelz, gyrox, gyroy, gyroz, irdata = np.zeros( (8, n_samples) )
    i = 0
    while i < n_samples :                      # while we still need samples, keep reading
        data = ""
        try:
            data = ser.read_until(',').decode('utf-8').strip()
            print (data)
            if (data == str(-2)): 
                connect()
                return time, accelx, accely, accelz, gyrox, gyroy, gyroz, irdata
             
            t, ax, ay, az, gx, gy, gz, ir = data.split(' ')
            t = float(t) / 1000
            ax = float(ax)
            ay = float(ay)
            az = float(az)
            gx = float(gx)
            gy = float(gy)
            gz = float(gz)
            ir = float(ir)
        except :                               # report error if we failed
            print('Invalid data: ', data)
            if i == 0:
                t = times[-1] + 1/f_samp* 1000 #fill in missing time slot
            #copy past values over to help avoid errors
                ax = accx[-1]
                ay = accy[-1]
                az = accz[-1]
                gx = gyx[-1]
                gy = gyy[-1]
                gz = gyz[-1]
                ir = IR_Data[-1]
            else:
                t = time[-1] + 1/f_samp* 1000 #fill in missing time slot
                #copy past values over to help avoid errors
                ax = accelx[-1]
                ay = accely[-1]
                az = accelz[-1]
                gx = gyrox[-1]
                gy = gyroy[-1]
                gz = gyroz[-1]
                ir = irdata[-1]
            continue
        
        time[:-1] = time[1:]                  # shift and update our time/values arrays
        time[-1] = t
        
        accelx[:-1] = accelx[1:]
        accelx[-1] = ax
        
        accely[:-1] = accely[1:]
        accely[-1] = ay
        
        accelz[:-1] = accelz[1:]
        accelz[-1] = az
        
        gyrox[:-1] = gyrox[1:]
        gyrox[-1] = gx
        
        gyroy[:-1] = gyroy[1:]
        gyroy[-1] = gy
        
        gyroz[:-1] = gyroz[1:]
        gyroz[-1] = gz
        
        irdata[:-1] = irdata[1:]
        irdata[-1] = ir
        
        i += 1

    sample_count += n_samples
    return time, accelx, accely, accelz, gyrox, gyroy, gyroz, irdata
    
# ==================== Grab new samples and plot ====================
def update_data(): #i is needed for live plotting
    global times, accx, accy, accz, gyx, gyy, gyz, IR_Data, filtered_vals, filt_ICs, steps
    

    # shift samples left by 'NS'
    times[:N-NS] = times[NS:]
    accx[:N-NS] = accx[NS:]
    accy[:N-NS] = accy[NS:]
    accz[:N-NS] = accz[NS:]
    gyx[:N-NS] = gyx[NS:]
    gyy[:N-NS] = gyy[NS:]
    gyz[:N-NS] = gyz[NS:]
    IR_Data[:N-NS] = IR_Data[NS:]
    
    times[N-NS:], accx[N-NS:], accy[N-NS:], accz[N-NS:], gyx[N-NS:], gyy[N-NS:], gyz[N-NS:], IR_Data[N-NS:]= grab_samples(NS)
    
    filtered_vals[0][:N-NS] = filtered_vals[0][NS:]
    filtered_vals[1][:N-NS] = filtered_vals[1][NS:]
    filtered_vals[2][:N-NS] = filtered_vals[2][NS:]
    filtered_vals[3][:N-NS] = filtered_vals[3][NS:]
    filtered_vals[4][:N-NS] = filtered_vals[4][NS:]
    filtered_vals[5][:N-NS] = filtered_vals[5][NS:]
    filtered_vals[6][:N-NS] = filtered_vals[6][NS:]
    
   
    filtered_vals[0][N-NS:], filt_ICs[0] = process_ir(accx[N-NS:], filt_coeffs, filt_ICs[0], 0)
    filtered_vals[1][N-NS:], filt_ICs[1] = process_ir(accy[N-NS:], filt_coeffs, filt_ICs[1], 0)
    filtered_vals[2][N-NS:], filt_ICs[2] = process_ir(accz[N-NS:], filt_coeffs, filt_ICs[2], 0)
    filtered_vals[3][N-NS:], filt_ICs[3] = process_ir(gyx[N-NS:], filt_coeffs, filt_ICs[3], 0)
    filtered_vals[4][N-NS:], filt_ICs[4] = process_ir(gyy[N-NS:], filt_coeffs, filt_ICs[4], 0)
    filtered_vals[5][N-NS:], filt_ICs[5] = process_ir(gyz[N-NS:], filt_coeffs, filt_ICs[5], 0)
    filtered_vals[6][N-NS:], filt_ICs[6] = process_ir(IR_Data[N-NS:], filt_coeffs, filt_ICs[6], 0)
    # grab new samples
    
    samp4welch = 30 / (times[-1] - times[-31])
    freqs, pwr = sig.welch(filtered_vals[5][-31:-1] + filtered_vals[4][-31:-1] + filtered_vals[3][-31:-1], samp4welch, nperseg = 30) 
    idx = np.argmax(pwr)
    fmax = freqs [idx]
    f_steps = fmax * 2
    if f_steps > 0.75 and f_steps < 4  and max(pwr) > 1e4:
        steps += f_steps * (times[-1] - times[-NS])
        
        if int(f_steps * (times[-1] - times[-NS])) != 0:
            ser.write(str(steps).encode('utf-8'))
            print(int(steps))
            print (max(pwr))
            print (f_steps)
            
# =============================================================================
#     axes.set_xlim(times[0],times[N-1])
#     live_plot.set_data(times, filtered_vals[6])
# =============================================================================

    return #live_plot


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
    filt_ICs[4][0] = zi_hi
    filt_ICs[5][0] = zi_hi
    filt_ICs[6][0] = zi_hi
    filt_ICs[0][1] = zi_low
    filt_ICs[1][1] = zi_low
    filt_ICs[2][1] = zi_low
    filt_ICs[3][1] = zi_low
    filt_ICs[4][1] = zi_low
    filt_ICs[5][1] = zi_low
    filt_ICs[6][1] = zi_low

# ==================== Main ====================
# Note: this assumes data is being sent by the Arduino in the format of "<time> <value>"
# You will need to modify it to suit your needs!
if (__name__ == "__main__") :
  
    # Open Serial
    with serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1) as ser:
        
        setup()
        
        build_filters()
          # ser.write(b'1')                         # tell Arduino to start sending data. NOTE!!: you should change this for your setup 
       
        times, accx, accy, accz, gyx, gyy, gyz, IR_Data = grab_samples(N) 
 
        filtered_vals[0], filt_ICs[0] = process_ir(accx, filt_coeffs, filt_ICs[0], 1)
        filtered_vals[1], filt_ICs[1] = process_ir(accy, filt_coeffs, filt_ICs[1], 1)
        filtered_vals[2], filt_ICs[2] = process_ir(accz, filt_coeffs, filt_ICs[2], 1)
        filtered_vals[3], filt_ICs[3] = process_ir(gyx, filt_coeffs, filt_ICs[3], 1)
        filtered_vals[4], filt_ICs[4] = process_ir(gyy, filt_coeffs, filt_ICs[4], 1)
        filtered_vals[5], filt_ICs[5] = process_ir(gyz, filt_coeffs, filt_ICs[5], 1)
        filtered_vals[6], filt_ICs[6] = process_ir(IR_Data, filt_coeffs, filt_ICs[6], 1)
        
        samp4welch = N / (times[-1] - times[0])
        freqs, pwr = sig.welch(filtered_vals[5] + filtered_vals[4] + filtered_vals[3], samp4welch, nperseg = N) 
        idx = np.argmax(pwr)
        max_pwr_old = np.max(pwr)
        fmax = freqs [idx]
        f_steps = fmax * 2
        if f_steps > 0.5 and f_steps < 4  and max(pwr) > 1e4 :
            steps = f_steps * times[-1]
        print(int (steps))
        ser.write((str(steps) ).encode ('utf-8')) #+ str(HR)
        
# =============================================================================
#         fig, axes = plt.subplots()
# 
#         live_plot = axes.plot(times, filtered_vals[6], lw=2)[0]
#         
#         anim = animation.FuncAnimation(fig, update_data, interval=1000)
#         plt.show()
#          
# =============================================================================
        while (True): 
            update_data()
            
