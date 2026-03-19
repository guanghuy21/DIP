import time
import pyvisa
import shutil
import math
import serial
import numpy as np
import csv
import os
from itertools import zip_longest
import winsound
import time
###################################################################################################
# User Parameters
addr = 'TCPIP0::localhost::hislip_PXI10_CHASSIS1_SLOT1_INDEX0::INSTR' # Device VISA address
VNA_model = 'P5021A'
Rotator_port = 'COM4'
test2 = False # If not using VNA then True
tolerances = [0.5,99,99,99] #Tr1 to 4
max_attempts=100 # max_attempt for each ascan 
b_scan_count = 72
name = 'minus_30deg_cross'
retry = False
###################################################################################################
# Define Helper Function
def alarm():
    # Define the frequency (Hz) and duration (ms) for the alarm sound
    frequency = 1000  # Adjust this value for a different pitch
    duration = 300  # Adjust this value for a longer or shorter beep

    # Number of beeps to create the alarm effect
    num_beeps = 5  # Adjust this value for more or fewer beeps

    for _ in range(num_beeps):
        winsound.Beep(frequency, duration)
        time.sleep(0.1)  # A short pause between beeps

def capture_and_process_data(vna, numtrace):
    trace_indices = ",".join(str(i) for i in range(1, numtrace + 1))
    vna.write("SENS:SWE:MODE SING")
    vna.query("*OPC?")

    query_string = f'CALC:DATA:MSD? "{trace_indices}"'
    data = vna.query_binary_values(query_string, datatype='d')

    traces = [data[len(data)//numtrace*i:len(data)//numtrace*(i+1)] for i in range(numtrace)]
    real_parts = np.array([trace[::2] for trace in traces])
    imag_parts = np.array([trace[1::2] for trace in traces])

    magnitudes = np.array([np.abs(np.array(real) + 1j * np.array(imag)) for real, imag in zip(real_parts, imag_parts)])

    return real_parts, imag_parts, magnitudes

def calculate_relative_error_percent(previous_magnitudes, magnitudes):
    re_percentages = []
    for prev_trace, current_trace in zip(previous_magnitudes, magnitudes):
        re = np.abs(current_trace - prev_trace)/np.array(current_trace)
        re_percentage = np.mean(re)*100  # Calculate RMSE as a percentage
        re_percentages.append(re_percentage)
    return re_percentages

def calculate_rmse(previous_magnitudes, magnitudes):
    rmses = []
    for prev_trace, current_trace in zip(previous_magnitudes, magnitudes):
        rmse = np.mean(np.square(current_trace - prev_trace))
        rmsep = 100*rmse/np.mean(prev_trace)
        rmses.append(rmsep)
    return rmses

# 2 RUN ONLY VERSION
def run_until_convergence(vna, numtrace=4, tolerances=None, max_attempts=20):
    if tolerances is None:
        tolerances = [1] * numtrace  # Default tolerance as a percentage for all traces

    previous_magnitudes = None
    previous_real = None
    previous_imag = None

    for attempt in range(max_attempts):
        print(f"Starting Attempt: {attempt + 1}")
        real, imag, magnitudes = capture_and_process_data(vna, numtrace)

        if previous_magnitudes is not None:
            # Calculate RMSE percentages for each trace
            e_percentages = calculate_relative_error_percent(previous_magnitudes, magnitudes)

            # Print the RMSE percentage for each trace
            for i, e_percentage in enumerate(e_percentages):
                print(f"Trace {i + 1}: Error% = {e_percentage:.2f}%")

            # Check if each trace has converged within its respective tolerance
            converged = all(rmse <= tol for rmse, tol in zip(e_percentages, tolerances))

            if converged:
                print(f"Converged after {attempt + 1} attempts.")
                # Calculate the mean of the current and previous real and imaginary data
                mean_real = [(r + prev_r)/2 for r, prev_r in zip(real, previous_real)]
                mean_imag = [(i + prev_i)/2 for i, prev_i in zip(imag, previous_imag)]
                return mean_real, mean_imag

        # If not converged, print a message and continue
        if attempt > 0:
            print(f"Attempt {attempt + 1}: Magnitudes not converged.")
        
        # Update the previous to the current values
        previous_magnitudes = magnitudes
        previous_real = real
        previous_imag = imag

    print("Maximum attempts reached. Convergence not achieved.")
    print("ERROR: Not enough data collected for B-scan")
    alarm()
    return None, None

def connect_to_vna(addr, VNA_model):
    try:
        print('\nAttempting to connect to: ' + addr)

        # Opening VISA resource manager:
        rm = pyvisa.ResourceManager()

        # Connect to equipment at my address 'addr' above
        vna = rm.open_resource(addr, timeout=5000)

        # Check what the device at this VISA address responds as:
        resp = vna.query('*IDN?')

        if VNA_model in resp:
            print('\nSuccessfully connected to Keysight VNA!\n')
            # Set format of instrument to return
            vna.write('FORMat REAL,64')

            # Set byte order to swapped (little-endian) format
            vna.write('FORMat:BORDer SWAP')

            # Set starting condition to HOLD
            vna.write("SENS:SWE:MODE HOLD")
            time.sleep(0.1)

            return vna
        else:
            print('\nUnable to connect to Keysight VNA.\n')
            return None

    except pyvisa.errors.VisaIOError as e:
        print(f'Error connecting to VNA: {e}')
        alarm()
        return None

def connect_to_arduino(comport):
    try:
        arduino = serial.Serial(comport)
        arduino.baudrate = 9600
        arduino.timeout = 5
        arduino.bytesize = 8
        arduino.parity = 'N'
        arduino.stopbits = 1
        print("Connected to Arduino via", comport)
        return arduino
    except serial.SerialException as e:
        print(f"An error occurred while connecting to Arduino via {comport}: {e}")
        alarm()
        return None

def bscan(count):
    angle = []
    step = 360/count
    for i in range(1, count+1):
        angle.append(i*step)
    angle = np.array(angle)

    return angle

def env_creation(env_name, start, retry_flag=False):
    idx = start
    last_folder = None
    while True:
        folder = f'{env_name}_{idx}'
        if not os.path.exists(folder):
            if retry_flag:
                return last_folder  # Return the last existing folder
            os.mkdir(folder)
            break
        idx += 1
        last_folder = folder

    return folder

def set_rotator(angle, rotator): #Currently the rotator command is R1->36
    print(f'Rotator angle: {round(angle)} degrees')
    rotator.write(f'R{round(angle)}'.encode("utf-8"))
    rotator.flushOutput()  # Flush the output buffer
    count = 0
    while count < 3:
        response = rotator.read()
        if response != b'b':
            count += 1
            print(f'Attempt {count}')
        else:
            break
    if count >= 10:
        print('ERROR: Cannot communicate with Rotator')
        alarm()
    rotator.flushInput()  # Flush the input buffer

def set_slider(slider):
    print(f'\nMoving to Dist: 50cm')
    command = f'P {round(50 * conversion[0])} {round(90 * conversion[1])}'.encode("utf-8")
    slider.write(command)
    slider.flushOutput()  # Flush the output buffer
    count = 0
    while count < 3:
        response = slider.read()
        if response != b'b':
            count += 1
            print(f'Attempt {count}')
        else:
            break
    if count >= 3:
        print('ERROR: Cannot communicate with Slider')
        alarm()
    slider.flushInput()  # Flush the input buffer

def reset_slider(slider):
    print('\nDone B-scan, Resetting Slider')
    slider.write(b'c')
    slider.flushOutput()  # Flush the output buffer
    count = 0
    while count < 10:
        response = slider.read()
        if response != b'b':
            count += 1
            print(f'Attempt {count}')
        else:
            break
    if count >= 10:
        print('ERROR: Cannot communicate with Slider')
        alarm()
    slider.flushInput()  # Flush the input buffer


def get_last_existing_number(folder_name):
    try:
        # Get a list of files in the specified folder
        files = os.listdir(folder_name)

        # Filter out only the CSV files
        csv_files = [file for file in files if (file.endswith('.csv') and file != "air.csv")]

        # Extract numeric part from each file name and convert to integers
        numbers = [int(file.split('.')[0]) for file in csv_files]

        # Return the maximum number
        if numbers:
            return max(numbers)
        else:
            return None  # No CSV files found in the folder
    except Exception as e:
        print(f"Error: {e}")
        return None
###################################################################################################
# Main Function
# Create the working directory, copy the air.csv and matlab code
# Establish the connections to the arduino and VNA

rotator = connect_to_arduino(Rotator_port)
vna = connect_to_vna(addr, VNA_model)
rot = bscan(b_scan_count)

input("Press Enter to start the program...")
start = time.time()

folder = env_creation(name, 1, retry_flag=retry)
print('Folder: ', folder)

if retry:
    last_number = get_last_existing_number(folder)
    if last_number is not None:
        print(f"Last trace recorded is {last_number}")
    else:
        print("No CSV files found in the folder. Start from interval 1")
        last_number = 0
else:
    last_number = 0

for i in range(last_number,b_scan_count):    
    set_rotator(rot[i],rotator)
    print(f"\nInterval: {(i + 1)}")
    starttime = time.time()

    if not test2:
        # Sweeping
        real, imag = run_until_convergence(vna, numtrace=4, tolerances = tolerances, max_attempts = max_attempts)
        if(real is None or imag is None):
            break

        #write data into csv
        data = [x for pair in zip(real, imag) for x in pair]
        export_data = zip_longest(*data, fillvalue='')
        with open(f'{folder}/{i+1}.csv', 'w', encoding="ISO-8859-1", newline='') as file:
            write = csv.writer(file)
            write.writerows(export_data)
        file.close()
    
    
    endtime = time.time()
    print("\nTime taken for each A-scan:%.2f seconds" %(endtime - starttime))

end = time.time()
print(f'\nTime taken for entire B-scan {end - start:.2f}')
alarm()

# Reset Arduino
rotator.close()
