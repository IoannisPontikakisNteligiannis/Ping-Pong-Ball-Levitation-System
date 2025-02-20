import serial
import time
import matplotlib.pyplot as plt
import numpy as np


ser = serial.Serial('COM3', 9600)  
time.sleep(2)  # Wait for the connection to establish

# Lists to store the data
current_position = []
setpoint = []
error = []
motor_output = []
time_stamps = []

# Collect data for 1 minute (60 seconds)
start_time = time.time()
while time.time() - start_time < 60:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()  # Read data from serial port
        print(f"Raw data: {data}")  # Debug: print the raw data
        try:
            # Expecting 4 values: current_position, setpoint, error, motor_output
            values = data.split(',')
            if len(values) == 4:
                # Add values to respective lists
                current_position.append(float(values[0]))
                setpoint.append(float(values[1]))
                error.append(float(values[2]))
                motor_output.append(float(values[3]))
                time_stamps.append(time.time() - start_time)
        except ValueError:
            print(f"Invalid data format: {values}")

# Close the serial connection after data collection
ser.close()

# Plotting the data
plt.figure(figsize=(15, 10))

# Position and Setpoint
plt.subplot(2, 3, 1)
plt.plot(time_stamps, current_position, label='Current Position')
plt.plot(time_stamps, setpoint, label='Setpoint', linestyle='--')
plt.title('Position Tracking')
plt.xlabel('Time (s)')
plt.ylabel('Position (cm)')
plt.legend()

# Error
plt.subplot(2, 3, 2)
plt.plot(time_stamps, error, color='red', label='Position Error')
plt.title('Position Error')
plt.xlabel('Time (s)')
plt.ylabel('Error (cm)')
plt.axhline(y=0, color='k', linestyle='--')
plt.legend()

# Motor Output
plt.subplot(2, 3, 3)
plt.plot(time_stamps, motor_output, color='green', label='Motor Output')
plt.title('Motor Output')
plt.xlabel('Time (s)')
plt.ylabel('PWM Value')
plt.legend()

# Error Distribution Histogram
plt.subplot(2, 3, 4)
plt.hist(error, bins=20, edgecolor='black')
plt.title('Error Distribution')
plt.xlabel('Error (cm)')
plt.ylabel('Frequency')

# Error Magnitude Over Time
plt.subplot(2, 3, 5)
plt.plot(time_stamps, np.abs(error), color='purple', label='Absolute Error')
plt.title('Absolute Error Magnitude')
plt.xlabel('Time (s)')
plt.ylabel('Absolute Error (cm)')
plt.legend()

# Performance Metrics
plt.subplot(2, 3, 6)
plt.text(0.5, 0.8, f'Mean Error: {np.mean(error):.2f} cm', horizontalalignment='center')
plt.text(0.5, 0.6, f'Max Error: {max(np.abs(error)):.2f} cm', horizontalalignment='center')
plt.text(0.5, 0.4, f'Std Dev Error: {np.std(error):.2f} cm', horizontalalignment='center')
plt.axis('off')
plt.title('Performance Metrics')

# Adjust layout and display
plt.tight_layout()
plt.show()