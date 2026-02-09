#!/usr/bin/env python
"""
| File: launch_fixedwing_simulation.py
| Author: [Your Name]
| License: BSD-3-Clause
| Description: Isaac Sim standalone app for fixed-wing aircraft simulation with Ardupilot/PX4 backend
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating vehicles
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS, ROBOTS
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (
    ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig
)
# Alternative: Ardupilot backend
# from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (
#     ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig
# )
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation

# Import the FixedWing class
from pegasus.simulator.logic.vehicles.fixedwing import FixedWing, FixedWingConfig


class FixedWingApp:
    """
    Isaac Sim standalone application for fixed-wing aircraft simulation
    """

    def __init__(self):
        """
        Initialize the FixedWingApp and setup the simulation environment
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World - controls physics, spawning assets, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load simulation environment
        # Options: "Curved Gridroom", "Default Environment", "Black Gridroom", "Hospital", "Office", "Warehouse"
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])

        # Create the fixed-wing aircraft
        self.create_fixedwing_vehicle()

        # Reset the simulation environment so that all articulations are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback
        self.stop_sim = False

        print("✓ Fixed-wing simulation initialized successfully!")

    def create_fixedwing_vehicle(self):
        """
        Create a single fixed-wing aircraft with configured backend
        """
        
        # ============================================
        # FIXED-WING CONFIGURATION
        # ============================================
        config = FixedWingConfig()
        
        # Propeller/Motor settings
        config.prop_max_thrust = 10000.0          # Maximum thrust in Newtons
        config.prop_max_rpm = 10000.0          # Maximum RPM
        config.prop_thrust_coefficient = 0.000075  # Thrust coefficient
        config.prop_rotation_dir = 1           # 1: CCW, -1: CW
        
        # Aircraft geometry
        config.wing_area = 0.65                # Wing area (m²)
        config.wing_span = 3.0                 # Wing span (m)
        config.chord = 0.22                    # Mean aerodynamic chord (m)
        
        # Aerodynamic coefficients (adjust based on your aircraft)
        config.CL_0 = 0.3                      # Zero AoA lift coefficient
        config.CL_alpha = 4.0                  # Lift curve slope
        config.CL_max = 1.5                    # Stall limit (upper)
        config.CD_0 = 0.025                    # Parasitic drag
        
        # Control surface effectiveness
        config.Cm_elevator = -1.5              # Elevator pitch moment
        config.Cl_aileron = 0.3                # Aileron roll moment
        config.Cn_rudder = -0.05               # Rudder yaw moment
        
        # ============================================
        # ALTERNATIVE: ARDUPILOT BACKEND (Uncomment to use)
        # ============================================
        ardupilot_config = ArduPilotMavlinkBackendConfig({
            "vehicle_id": 0,
            "ardupilot_autolaunch": True,
            "ardupilot_dir": self.pg.ardupilot_path,
            "ardupilot_vehicle_model": "plane",
            "ardupilot_vehicle" : "ArduPlane"
        })
        
        # ============================================
        # ROS2 BACKEND (Optional - for ROS2 integration)
        # ============================================
        ros2_config = {
            "namespace": "fixedwing",
            "pub_sensors": True,
            "pub_graphical_sensors": True,
            "pub_state": True,
            "sub_control": False,
            "pub_tf": True,
        }
        
        # Combine backends
        config.backends = [
            ArduPilotMavlinkBackend(config=ardupilot_config),  # Uncomment for Ardupilot
            # ROS2Backend(vehicle_id=0, config=ros2_config)        # Optional ROS2
        ]
        
        # ============================================
        # CREATE THE AIRCRAFT
        # ============================================
        self.aircraft = FixedWing(
            stage_prefix="/World/fixedwing0",
            usd_file=ROBOTS['Fixed Wing'],
            vehicle_id=0,
            init_pos=[0.0, 0.0, 0.2],                    # Start 0.2m above ground
            init_orientation=Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config
        )
        
        print(f"✓ Fixed-wing aircraft created at position [0.0, 0.0, 0.5]")

    def run(self):
        """
        Main application loop - executes physics steps
        """

        # Start the simulation
        self.timeline.play()
        print("▶ Simulation started!")

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI and perform physics step
            self.world.step(render=True)
            
            # Optional: Print aerodynamic state every 100 steps (for debugging)
            # if self.world.current_time_step_index % 100 == 0:
            #     aero_state = self.aircraft.get_aerodynamic_state()
            #     print(f"Airspeed: {aero_state['airspeed']:.2f} m/s, "
            #           f"AoA: {aero_state['angle_of_attack']:.1f}°, "
            #           f"Throttle: {aero_state['throttle']:.0%}")
        
        # Cleanup and stop
        carb.log_warn("Fixed-wing Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    """
    Main entry point
    """
    # Instantiate the app
    app = FixedWingApp()

    # Run the application loop
    app.run()


if __name__ == "__main__":
    main()