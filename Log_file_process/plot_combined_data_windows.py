import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def parse_flight_log(file_path):
    """
    Parse the flight log file according to the format:
    %u:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%.2f,%.2f\r\n
    
    Parameters correspond to:
    0 - time
    1 - my_ahrs.Angle_Data.roll
    2 - my_ahrs.Angle_Data.pitch
    3 - my_ahrs.Angle_Data.yaw
    4 - my_ahrs.IMU_Data.gyro.x
    5 - my_ahrs.IMU_Data.gyro.y
    6 - my_ahrs.IMU_Data.gyro.z
    7 - my_aircraft.Altitude
    8 - TOF.distance_m
    9 - my_aircraft.Temperature
    10 - my_aircraft.Battery_Volt*0.1f
    11 - my_aircraft.Throttle
    12 - AttitudeControl.pitch_target_angle
    13 - AttitudeControl.roll_target_angle
    14 - AttitudeControl.yaw_target_angle
    15 - HeightControl.target_height
    """
    
    # Define column names for easier reference
    column_names = [
        'time',
        'roll', 'pitch', 'yaw',
        'gyro_x', 'gyro_y', 'gyro_z',
        'altitude', 'tof_distance', 'temperature',
        'battery_volt', 'throttle',
        'target_pitch', 'target_roll', 'target_yaw',
        'target_height'
    ]
    
    # Read the file
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    # Filter out the header line if present
    data_lines = []
    for line in lines:
        line = line.strip()
        if line and not line.startswith('----'):
            data_lines.append(line)
    
    # Parse each data line
    parsed_data = []
    for line in data_lines:
        try:
            # Split by comma first
            parts = line.split(',')
            if len(parts) < 15:
                continue
                
            # The first part is "time:roll", so split that to get time and roll separately
            first_part_split = parts[0].split(':')
            if len(first_part_split) != 2:
                continue  # Skip if the first part doesn't have a colon
            
            time = int(first_part_split[0])
            roll = float(first_part_split[1])

            # Now build values array: [time, roll, pitch, yaw, ...]
            values = [time, roll]

            # Add the rest of the values (pitch through target_height)
            for i in range(1, len(parts)):  # Start from index 1 since we already handled roll
                # Clean the value by removing possible carriage returns or newlines
                cleaned_value = parts[i].strip()
                if cleaned_value.endswith('\\r\\n') or cleaned_value.endswith('\\n'):
                    cleaned_value = cleaned_value[:-4]  # Remove \r\n or \n
                elif cleaned_value.endswith('\\r'):
                    cleaned_value = cleaned_value[:-2]  # Remove \r
                try:
                    values.append(float(cleaned_value))
                except ValueError:
                    # If parsing fails, use 0.0 as default
                    values.append(0.0)
            
            # Ensure we have all expected 16 values
            if len(values) < 16:
                # Pad with zeros if necessary
                values.extend([0.0] * (16 - len(values)))
            
            parsed_data.append(values)
        except Exception as e:
            continue  # Skip lines that can't be parsed
    
    # Convert to numpy array
    if parsed_data:
        data_array = np.array(parsed_data)
        
        # Return the data organized by column name
        result = {}
        for i, name in enumerate(column_names):
            if len(data_array) > 0:
                result[name] = data_array[:, i]
            else:
                result[name] = np.array([])
        
        return result
    else:
        # Return empty arrays if no valid data was parsed
        return {name: np.array([]) for name in column_names}

def create_combined_plots(data, filename="LOG.TXT"):
    """
    Create combined plots in windows as requested by the user
    """
    time = data['time']

    if len(time) == 0:
        print("No data to plot!")
        return
    
    # # Create figure for attitude comparison (roll vs target_roll)
    # plt.figure(figsize=(12, 6))
    # plt.plot(time, data['roll'], label='Actual Roll', color='red', linestyle='-')
    # plt.plot(time, data['target_roll'], label='Target Roll', color='orange', linestyle='--')
    # plt.title('Roll Angle vs Target Roll Angle')
    # plt.xlabel('Time (ms)')
    # plt.ylabel('Roll Angle (°)')
    # plt.legend()
    # plt.grid(True)
    # plt.tight_layout()

    # # Create figure for pitch comparison (pitch vs target_pitch)
    # plt.figure(figsize=(12, 6))
    # plt.plot(time, data['pitch'], label='Actual Pitch', color='green', linestyle='-')
    # plt.plot(time, data['target_pitch'], label='Target Pitch', color='lightgreen', linestyle='--')
    # plt.title('Pitch Angle vs Target Pitch Angle')
    # plt.xlabel('Time (ms)')
    # plt.ylabel('Pitch Angle (°)')
    # plt.legend()
    # plt.grid(True)
    # plt.tight_layout()

    # # Create figure for yaw comparison (yaw vs target_yaw)
    # plt.figure(figsize=(12, 6))
    # plt.plot(time, data['yaw'], label='Actual Yaw', color='blue', linestyle='-')
    # plt.plot(time, data['target_yaw'], label='Target Yaw', color='lightblue', linestyle='--')
    # plt.title('Yaw Angle vs Target Yaw Angle')
    # plt.xlabel('Time (ms)')
    # plt.ylabel('Yaw Angle (°)')
    # plt.legend()
    # plt.grid(True)
    # plt.tight_layout()
    
    # # Create figure for gyroscope data
    # plt.figure(figsize=(12, 6))
    # plt.plot(time, data['gyro_x'], label='Gyro X', color='red')
    # plt.plot(time, data['gyro_y'], label='Gyro Y', color='green')
    # plt.plot(time, data['gyro_z'], label='Gyro Z', color='blue')
    # plt.title('Gyroscope Data')
    # plt.xlabel('Time (ms)')
    # plt.ylabel('Angular Velocity (°/s)')
    # plt.legend()
    # plt.grid(True)
    # plt.tight_layout()
    
    # # Create figure for TOF and target height
    # fig, ax1 = plt.subplots(figsize=(12, 6))
    
    # color = 'tab:blue'
    # ax1.set_xlabel('Time (ms)')
    # ax1.set_ylabel('TOF Distance (m)', color=color)
    # line1 = ax1.plot(time, data['tof_distance'], label='TOF Distance', color=color)
    # ax1.tick_params(axis='y', labelcolor=color)
    # ax1.grid(True)
    
    # ax2 = ax1.twinx()
    # color = 'tab:orange'
    # ax2.set_ylabel('Target Height (m)', color=color)
    # line2 = ax2.plot(time, data['target_height'], label='Target Height', color=color)
    # ax2.tick_params(axis='y', labelcolor=color)
    
    # # Combine legends from both axes
    # lines1, labels1 = ax1.get_legend_handles_labels()
    # lines2, labels2 = ax2.get_legend_handles_labels()
    # ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    # plt.title('TOF Distance and Target Height')
    # fig.tight_layout()
    
    # # Create figure for throttle and altitude
    # fig, ax1 = plt.subplots(figsize=(12, 6))
    
    # color = 'tab:purple'
    # ax1.set_xlabel('Time (ms)')
    # ax1.set_ylabel('Throttle (%)', color=color)
    # line1 = ax1.plot(time, data['throttle'], label='Throttle', color=color)
    # ax1.tick_params(axis='y', labelcolor=color)
    # ax1.grid(True)
    
    # ax2 = ax1.twinx()
    # color = 'tab:olive'
    # ax2.set_ylabel('Altitude (m)', color=color)
    # line2 = ax2.plot(time, data['altitude'], label='Altitude', color=color)
    # ax2.tick_params(axis='y', labelcolor=color)
    
    # # Combine legends from both axes
    # lines1, labels1 = ax1.get_legend_handles_labels()
    # lines2, labels2 = ax2.get_legend_handles_labels()
    # ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    # plt.title('Throttle and Altitude')
    # fig.tight_layout()
    
    # Create an overview plot with all key data
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle(f'Flight Data Overview - {filename}', fontsize=16)
    
    # Plot 1: Roll comparison
    axes[0, 0].plot(time, data['roll'], label='Actual Roll', color='red', linestyle='-')
    axes[0, 0].plot(time, data['target_roll'], label='Target Roll', color='orange', linestyle='--')
    axes[0, 0].set_title('Roll Angle Comparison')
    axes[0, 0].set_xlabel('Time (ms)')
    axes[0, 0].set_ylabel('Roll Angle (°)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # Plot 2: Pitch comparison
    axes[0, 1].plot(time, data['pitch'], label='Actual Pitch', color='green', linestyle='-')
    axes[0, 1].plot(time, data['target_pitch'], label='Target Pitch', color='lightgreen', linestyle='--')
    axes[0, 1].set_title('Pitch Angle Comparison')
    axes[0, 1].set_xlabel('Time (ms)')
    axes[0, 1].set_ylabel('Pitch Angle (°)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Plot 3: Yaw comparison
    axes[1, 0].plot(time, data['yaw'], label='Actual Yaw', color='blue', linestyle='-')
    axes[1, 0].plot(time, data['target_yaw'], label='Target Yaw', color='lightblue', linestyle='--')
    axes[1, 0].set_title('Yaw Angle Comparison')
    axes[1, 0].set_xlabel('Time (ms)')
    axes[1, 0].set_ylabel('Yaw Angle (°)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # Plot 4: Gyroscope data
    axes[1, 1].plot(time, data['gyro_x'], label='Gyro X', color='red')
    axes[1, 1].plot(time, data['gyro_y'], label='Gyro Y', color='green')
    axes[1, 1].plot(time, data['gyro_z'], label='Gyro Z', color='blue')
    axes[1, 1].set_title('Gyroscope Data')
    axes[1, 1].set_xlabel('Time (ms)')
    axes[1, 1].set_ylabel('Angular Velocity (°/s)')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # Plot 5: TOF distance and target height
    axes[2, 0].plot(time, data['tof_distance'], label='TOF Distance', color='tab:blue')
    axes[2, 0].plot(time, data['target_height'], label='Target Height', color='tab:orange')
    axes[2, 0].set_title('TOF Distance and Target Height')
    axes[2, 0].set_xlabel('Time (ms)')
    axes[2, 0].set_ylabel('Distance (m)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # Plot 6: Throttle and altitude
    ax_twin = axes[2, 1].twinx()
    p1, = axes[2, 1].plot(time, data['throttle'], label='Throttle', color='tab:purple')
    p2, = ax_twin.plot(time, data['altitude'], label='Altitude', color='tab:olive')
    axes[2, 1].set_title('Throttle and Altitude')
    axes[2, 1].set_xlabel('Time (ms)')
    axes[2, 1].set_ylabel('Throttle (%)', color='tab:purple')
    ax_twin.set_ylabel('Altitude (m)', color='tab:olive')
    axes[2, 1].legend([p1, p2], ['Throttle', 'Altitude'])
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    
    # Display all plots
    plt.show()
    
    print("Plots displayed successfully!")
    print(f"Total data points: {len(time)}")
    print(f"Time range: {time[0]:.1f} to {time[-1]:.1f} ms")

def main():
    # Get the current working directory and look for the log file there
    import os
    log_file_path = Path(os.getcwd()) / "LOG_TEST.TXT"  # Change to your log file name here

    if not log_file_path.exists():
        print(f"Log file {log_file_path} not found!")
        return
    
    print("Parsing flight log data...")
    data = parse_flight_log(log_file_path)
    
    print("Data loaded successfully!")
    print(f"Total data points: {len(data['time'])}")
    if len(data['time']) > 0:
        print(f"Time range: {data['time'][0]:.1f} to {data['time'][-1]:.1f} ms")
    
    # Create the combined plots
    create_combined_plots(data, log_file_path.name)

if __name__ == "__main__":
    main()