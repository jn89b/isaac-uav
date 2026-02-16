import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import numpy as np

# File to read
LOG_FILE = "forces_log.csv"


def _ensure_numeric_columns(df, columns):
    """Convert existing columns to numeric, creating missing ones as NaN."""
    for col in columns:
        if col not in df.columns:
            df[col] = np.nan
        df[col] = pd.to_numeric(df[col], errors="coerce")


def animate(i):
    if not os.path.exists(LOG_FILE):
        return

    try:
        # 1. Read the file
        df = pd.read_csv(LOG_FILE)
        
        if df.empty:
            return

        # 2. DATA CLEANING: Force columns to numeric and handle the 'thrust' list string
        # If thrust was logged as a string "[x, y, z]", we extract the first value (magnitude)
        if df['thrust'].dtype == object:
            df['thrust'] = df['thrust'].str.replace(r'[\[\]]', '', regex=True).str.split(',').str[0]

        # Convert columns to float, turning errors (like headers or empty strings) into NaN
        base_cols = ['timestamp', 'aot', 'sideslip', 'thrust', 'lift', 'drag', 'side', 'mx', 'my', 'mz', 'u', 'v', 'w']
        extra_cols = ['roll_deg', 'pitch_deg', 'yaw_deg', 'p', 'q', 'r']
        _ensure_numeric_columns(df, base_cols + extra_cols)

        # Drop any rows that failed to convert (like the header being repeated)
        df = df.dropna(subset=['timestamp'])

        # 3. Filter for display
        data = df.tail(500)

        # 4. Clear and Update Plots
        for ax in [ax0, ax1, ax2, ax3, ax4]:
            ax.clear()
        
        # Convert rad to deg safely
        # Using .values ensures we are passing a numpy array to np.degrees
        aot_deg = np.degrees(data['aot'].values).round(2)
        sideslip_deg = np.degrees(data['sideslip'].values).round(2)

        # Plot 0: Angles
        ax0.plot(data['timestamp'], aot_deg, label='AoA (deg)', color='red')
        # ax0.plot(data['timestamp'], sideslip_deg, label='Sideslip (deg)', color='blue')
        ax0.set_title('Aircraft Angles')
        ax0.set_ylabel('Degrees')
        ax0.legend(loc='upper left')
        ax0.grid(True)

        # Plot 1: Forces
        ax1.plot(data['timestamp'], data['lift'], label='Lift', color='blue')
        ax1.plot(data['timestamp'], data['drag'], label='Drag', color='red')
        ax1.plot(data['timestamp'], data['thrust'], label='Thrust (X)', color='orange', linestyle='--')
        ax1.set_title('Aerodynamic Forces & Thrust')
        ax1.set_ylabel('Force (N)')
        ax1.legend(loc='upper left')
        ax1.grid(True)

        # Plot 2: Moments
        ax2.plot(data['timestamp'], data['mx'], label='Roll (Mx)', color='purple')
        ax2.plot(data['timestamp'], data['my'], label='Pitch (My)', color='brown')
        ax2.plot(data['timestamp'], data['mz'], label='Yaw (Mz)', color='cyan')
        ax2.set_title('Aerodynamic Moments')
        ax2.set_ylabel('Moment (Nm)')
        ax2.legend(loc='upper left')
        ax2.grid(True)

        # Plot 3: Velocities
        ax3.plot(data['timestamp'], data['u'], label='u (fwd)', color='blue')
        ax3.plot(data['timestamp'], data['v'], label='v (lat)', color='red')
        ax3.plot(data['timestamp'], data['w'], label='w (vert)', color='green')
        ax3.set_title('Body Velocities')
        ax3.set_ylabel('Velocity (m/s)')
        ax3.legend(loc='upper left')
        ax3.grid(True)

        # Plot 4: Attitude and Rates (if present in CSV)
        if data[['roll_deg', 'pitch_deg', 'yaw_deg']].notna().any().any():
            ax4.plot(data['timestamp'], data['roll_deg'], label='Roll (deg)', color='purple')
            ax4.plot(data['timestamp'], data['pitch_deg'], label='Pitch (deg)', color='brown')
            ax4.plot(data['timestamp'], data['yaw_deg'], label='Yaw (deg)', color='cyan')

        if data[['p', 'q', 'r']].notna().any().any():
            ax4.plot(data['timestamp'], np.degrees(data['p']), label='p (deg/s)', color='purple', linestyle='--')
            ax4.plot(data['timestamp'], np.degrees(data['q']), label='q (deg/s)', color='brown', linestyle='--')
            ax4.plot(data['timestamp'], np.degrees(data['r']), label='r (deg/s)', color='cyan', linestyle='--')

        ax4.set_title('Attitude & Body Rates')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('deg, deg/s')
        ax4.legend(loc='upper left')
        ax4.grid(True)

    except Exception as e:
        # Use a print to debug if a specific row is still breaking the logic
        print(f"Plotting Error: {e}")

# Setup Figure
fig, (ax0, ax1, ax2, ax3, ax4) = plt.subplots(5, 1, sharex=True, figsize=(11, 12))

# Start Animation
ani = animation.FuncAnimation(fig, animate, interval=200, cache_frame_data=False)

plt.tight_layout()
print(f"Monitoring {LOG_FILE} for real-time aero data...")
plt.show()
