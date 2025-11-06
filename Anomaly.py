import json
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

LOG_FILE = "logging/velocity-voltage_log.json"

# Thresholds for velocity-to-voltage ratio
RATIO_THRESHOLD = (0.1, 2.0)  # Example range for acceptable ratios
EPSILON = 1e-6  # Small value to avoid division by zero (0.000001)

def read_log_file(file_path):
    """Read and parse the JSON log file."""
    try:
        with open(file_path, "r") as file:
            data = []
            for line_num, line in enumerate(file.readlines(), 1):
                line = line.strip()
                if not line:  # Skip empty lines
                    continue
                try:
                    data.append(json.loads(line))
                except json.JSONDecodeError as e:
                    print(f"Warning: Invalid JSON on line {line_num}: {e}")
                    continue
            return data
    except FileNotFoundError:
        print(f"Log file {file_path} not found.")
        return []
    except Exception as e:
        print(f"Error reading log file: {e}")
        return []

def detect_anomalies(data):
    """Detect anomalies in the velocity-to-voltage ratio."""
    anomalies = []
    
    if not data:
        print("No data to analyze.")
        return anomalies
    
    for i, entry in enumerate(data):
        try:
            velocity = entry.get("velocity", 0)
            voltage = entry.get("voltage", 1)  # Avoid division by zero
            timestamp = entry.get("timestamp", f"entry_{i}")

            # Handle potential string/numeric conversion for values
            try:
                velocity = float(velocity)
                voltage = float(voltage)
            except (ValueError, TypeError):
                print(f"Warning: Invalid numeric values at {timestamp}")
                continue

            # Calculate velocity-to-voltage ratio using epsilon for near-zero voltages
            if abs(voltage) < EPSILON:
                ratio = velocity / EPSILON
            else:
                ratio = velocity / voltage
            
            # Check if the ratio is outside the acceptable range
            if not (RATIO_THRESHOLD[0] <= ratio <= RATIO_THRESHOLD[1]):
                anomalies.append(f"Anomaly detected at {timestamp}: ratio={ratio:.3f}, velocity={velocity:.3f}, voltage={voltage:.3f}")
                
        except Exception as e:
            print(f"Error processing entry {i}: {e}")
            continue
            
    return anomalies

def plot_ratio(data):
    """Plot the velocity-to-voltage ratio over time."""
    if not data:
        print("No data to plot.")
        return
    
    timestamps = []
    ratios = []
    
    for i, entry in enumerate(data):
        try:
            velocity = float(entry.get("velocity", 0))
            voltage = float(entry.get("voltage", 1))
            timestamp = entry.get("timestamp", i)
            
            # Handle different timestamp formats
            if isinstance(timestamp, str):
                try:
                    # Try to parse as datetime if it's a string
                    timestamp = datetime.fromisoformat(timestamp.replace('Z', '+00:00')).timestamp()
                except:
                    # If parsing fails, use the index
                    timestamp = i
            
            # Calculate ratio, handling near-zero voltages with epsilon
            if abs(voltage) < EPSILON:
                ratio = velocity / EPSILON  # Use epsilon instead of actual voltage
            else:
                ratio = velocity / voltage
            
            timestamps.append(timestamp)
            ratios.append(ratio)
            
        except (ValueError, TypeError) as e:
            print(f"Warning: Skipping invalid entry {i}: {e}")
            continue

    if not timestamps:
        print("No valid data points to plot.")
        return

    # Create the plot
    plt.figure(figsize=(12, 8))
    
    # Plot the ratio, handling NaN values
    valid_indices = ~np.isnan(ratios)
    plt.plot(np.array(timestamps)[valid_indices], np.array(ratios)[valid_indices], 
             'b-', label="Velocity-to-Voltage Ratio", linewidth=1.5)
    
    # Highlight anomalous points
    anomalous_timestamps = []
    anomalous_ratios = []
    
    for i, (ts, ratio) in enumerate(zip(timestamps, ratios)):
        if not np.isnan(ratio) and not (RATIO_THRESHOLD[0] <= ratio <= RATIO_THRESHOLD[1]):
            anomalous_timestamps.append(ts)
            anomalous_ratios.append(ratio)
    
    if anomalous_timestamps:
        plt.scatter(anomalous_timestamps, anomalous_ratios, 
                   color='red', s=50, label="Anomalies", zorder=5)
    
    # Add threshold lines
    plt.axhline(RATIO_THRESHOLD[0], color="red", linestyle="--", 
                label=f"Lower Threshold ({RATIO_THRESHOLD[0]})", alpha=0.7)
    plt.axhline(RATIO_THRESHOLD[1], color="red", linestyle="--", 
                label=f"Upper Threshold ({RATIO_THRESHOLD[1]})", alpha=0.7)
    
    plt.xlabel("Time")
    plt.ylabel("Velocity-to-Voltage Ratio")
    plt.title("FRC Match Log: Velocity-to-Voltage Ratio Analysis")
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Add some statistics
    valid_ratios = np.array(ratios)[valid_indices]
    if len(valid_ratios) > 0:
        mean_ratio = np.mean(valid_ratios)
        std_ratio = np.std(valid_ratios)
        plt.figtext(0.02, 0.02, f"Mean: {mean_ratio:.3f}, Std: {std_ratio:.3f}, Points: {len(valid_ratios)}", 
                   fontsize=10, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray"))
    
    plt.tight_layout()
    plt.show()

def analyze_additional_metrics(data):
    """Analyze additional metrics that might indicate problems."""
    if not data:
        return
    
    print("\n=== Additional Analysis ===")
    
    velocities = []
    voltages = []
    ratios = []
    
    for entry in data:
        try:
            vel = float(entry.get("velocity", 0))
            volt = float(entry.get("voltage", 0))
            
            # Calculate ratio using same epsilon logic as main analysis
            if abs(volt) < EPSILON:
                ratio = vel / EPSILON
            else:
                ratio = vel / volt
            
            velocities.append(vel)
            voltages.append(volt)
            ratios.append(ratio)
        except (ValueError, TypeError):
            continue
    
    if velocities and voltages and ratios:
        vel_array = np.array(velocities)
        volt_array = np.array(voltages)
        ratio_array = np.array(ratios)
        
        print(f"Velocity - Mean: {np.mean(vel_array):.3f}, Std: {np.std(vel_array):.3f}, Range: [{np.min(vel_array):.3f}, {np.max(vel_array):.3f}]")
        print(f"Voltage - Mean: {np.mean(volt_array):.3f}, Std: {np.std(volt_array):.3f}, Range: [{np.min(volt_array):.3f}, {np.max(volt_array):.3f}]")
        print(f"Ratio - Mean: {np.mean(ratio_array):.3f}, Std: {np.std(ratio_array):.3f}, Range: [{np.min(ratio_array):.3f}, {np.max(ratio_array):.3f}]")
        
        # Check for potential issues
        zero_voltage_count = np.sum(np.abs(volt_array) < EPSILON)
        if zero_voltage_count > 0:
            print(f"Warning: {zero_voltage_count} entries with near-zero voltage detected!")
        
        # Check for sudden changes (basic derivative analysis)
        if len(vel_array) > 1:
            vel_diff = np.abs(np.diff(vel_array))
            large_changes = np.sum(vel_diff > 2 * np.std(vel_diff))
            if large_changes > 0:
                print(f"Info: {large_changes} sudden velocity changes detected (may indicate control issues)")

def main():
    print("FRC Match Log Anomaly Detector")
    print("=" * 40)
    
    # Read the velocity log file
    data = read_log_file(LOG_FILE)
    
    if not data:
        print("No valid data found. Please check your log file.")
        return
    
    print(f"Loaded {len(data)} log entries from {LOG_FILE}")
    
    # Detect anomalies
    anomalies = detect_anomalies(data)
    
    # Print anomalies
    if anomalies:
        print(f"\n{len(anomalies)} anomalies detected:")
        for anomaly in anomalies:
            print(f"  • {anomaly}")
    else:
        print("\nNo anomalies detected - all ratios within acceptable range!")
    
    # Additional analysis
    analyze_additional_metrics(data)
    
    # Plot the velocity-to-voltage ratio
    print("\nGenerating plot...")
    plot_ratio(data)

if __name__ == "__main__":
    main()