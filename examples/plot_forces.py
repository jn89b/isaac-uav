import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import sys

# File to read
LOG_FILE = "forces_log.csv"

def animate(i):
    if not os.path.exists(LOG_FILE):
        print(f"Waiting for {LOG_FILE} to be created...")
        return

    try:
        # Read the last N lines to avoid reading the whole file as it grows
        # or just read the whole file if it's not too huge. 
        # For Sim simplicity, let's read the whole file but limit the plot to last N seconds.
        df = pd.read_csv(LOG_FILE)
        
        if df.empty:
            return

        # Limit to last 500 points for performance
        data = df.tail(500)

        # Clear axes
        ax1.clear()
        ax2.clear()
        ax3.clear()

        # Plot Forces
        ax1.plot(data['timestamp'], data['lift'], label='Lift', color='blue')
        ax1.plot(data['timestamp'], data['drag'], label='Drag', color='red')
        ax1.plot(data['timestamp'], data['side'], label='Side', color='green')
        ax1.plot(data['timestamp'], data['thrust'], label='Thrust', color='orange', linestyle='--')
        
        ax1.set_title('Aerodynamic Forces & Thrust')
        ax1.set_ylabel('Force (N)')
        ax1.legend(loc='upper left')
        ax1.grid(True)

        # Plot Moments
        ax2.plot(data['timestamp'], data['mx'], label='Roll (Mx)', color='purple')
        ax2.plot(data['timestamp'], data['my'], label='Pitch (My)', color='brown')
        ax2.plot(data['timestamp'], data['mz'], label='Yaw (Mz)', color='cyan')

        ax2.set_title('Aerodynamic Moments')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Moment (Nm)')
        ax2.legend(loc='upper left')
        ax2.grid(True)

        # Plot Velocities
        ax3.plot(data['timestamp'], data['u'], label='u', color='blue')
        ax3.plot(data['timestamp'], data['v'], label='v', color='red')
        ax3.plot(data['timestamp'], data['w'], label='w', color='green')
        
        ax3.set_title('Velocities')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Velocity (m/s)')
        ax3.legend(loc='upper left')
        ax3.grid(True)
    except Exception as e:
        print(f"Error reading file: {e}")

# Setup Figure
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

# Start Animation
ani = animation.FuncAnimation(fig, animate, interval=100, cache_frame_data=False) # Update every 100ms

plt.tight_layout()
print(f"Monitoring {LOG_FILE}...")
plt.show()
