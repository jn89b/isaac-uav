"""
| File: fixedwing.py
| Author: [Your Name]
| License: BSD-3-Clause
| Description: Definition of the FixedWing class which is used as the base for all fixed-wing aircraft.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from omni.isaac.dynamic_control import _dynamic_control

# The vehicle interface
from pegasus.simulator.logic.vehicles.vehicle import Vehicle

# Mavlink interface (veya kendi backend'iniz)
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig

# Sensors
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS

# Dynamics
from pegasus.simulator.logic.dynamics import LinearDrag

# Force UI for coordinate debugging
from pegasus.simulator.ui.force_debugger import ForceControlWindow, DebugVisualizer

# Log
import carb
import csv
import os
import time

class FixedWingConfig:
    """
    A data class that is used for configuring a Fixed-Wing Aircraft
    """

    def __init__(self):
        """
        Initialization of the FixedWingConfig class
        """

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "fixedwing"

        # The USD file that describes the visual aspect of the vehicle
        self.usd_file = "../../assets/Robots/yoda_fixed_wing/yoda_fixed_wing.usd"

        # ============================================
        # PROPELLER/MOTOR CONFIGURATION
        # ============================================
        self.prop_max_thrust = 50.0  # Maximum thrust in Newtons
        self.prop_max_rpm = 8000.0   # Maximum RPM
        self.prop_thrust_coefficient = 0.00001  # Thrust = coef * RPM^2
        
        # Motor rotation direction (1: CCW, -1: CW when viewed from behind)
        self.prop_rotation_dir = 1

        # ============================================
        # AERODYNAMIC COEFFICIENTS
        # ============================================
        # Lift coefficients
        self.CL_0 = 0.28       # Zero angle of attack lift coefficient
        self.CL_alpha = 3.45   # Lift curve slope (per radian)
        self.CL_max = 1.4      # Maximum lift coefficient (stall limit)
        self.CL_min = -1.1     # Minimum lift coefficient
        
        # Drag coefficients
        self.CD_0 = 0.03       # Zero-lift drag coefficient (parasitic drag)
        self.CD_alpha = 0.30   # Drag due to angle of attack
        self.CD_alpha2 = 2.0   # Induced drag factor (quadratic term)
        
        # Side force coefficients
        self.CY_beta = -0.98   # Side force due to sideslip
        
        # Pitch moment coefficients
        self.Cm_0 = -0.02      # Zero angle of attack pitch moment
        self.Cm_alpha = -0.38  # Pitch moment curve slope
        
        # Roll moment coefficients  
        self.Cl_beta = -0.12   # Roll moment due to sideslip (dihedral effect)
        
        # Yaw moment coefficients
        self.Cn_beta = 0.25    # Yaw moment due to sideslip (weathercock stability)

        # ============================================
        # CONTROL SURFACE DERIVATIVES
        # ============================================
        # Elevator effectiveness (pitch control)
        self.CL_elevator = 0.43    # Lift change per elevator deflection
        self.Cm_elevator = -1.122  # Pitch moment per elevator deflection
        
        # Aileron effectiveness (roll control)
        self.Cl_aileron = 0.229    # Roll moment per aileron deflection
        
        # Rudder effectiveness (yaw control)
        self.Cn_rudder = -0.032    # Yaw moment per rudder deflection
        self.CY_rudder = 0.870     # Side force per rudder deflection

        # ============================================
        # AIRCRAFT GEOMETRY
        # ============================================
        self.wing_area = 0.55          # Wing area in m^2
        self.wing_span = 2.8           # Wing span in m
        self.chord = 0.18880           # Mean aerodynamic chord in m
        self.air_density = 1.225       # Air density in kg/m^3 (sea level)

        # ============================================
        # DRAG CONFIGURATION
        # ============================================
        # Linear drag (for body frame drag effects)
        self.drag = LinearDrag([0.1, 0.1, 0.1])

        # ============================================
        # SENSORS
        # ============================================
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
        self.graphical_sensors = []
        self.graphs = []

        # ============================================
        # BACKENDS
        # ============================================
        # Control backend (Mavlink, ROS2, or custom)
        self.backends = []


class FixedWing(Vehicle):
    """
    FixedWing class - Defines a base interface for creating fixed-wing aircraft
    """
    
    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "fixedwing",
        usd_file: str = "",
        vehicle_id: int = 0,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.5],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=FixedWingConfig(),
    ):
        """
        Initializes the fixed-wing aircraft object

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator. Defaults to "fixedwing".
            usd_file (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            vehicle_id (int): The id to be used for the vehicle. Defaults to 0.
            init_pos (list): Initial position in inertial frame (ENU). Defaults to [0.0, 0.0, 0.5].
            init_orientation (list): Initial orientation quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
            config (FixedWingConfig): Configuration object. Defaults to FixedWingConfig().
        """

        # 1. Initialize the Vehicle base class
        super().__init__(
            stage_prefix, 
            usd_file, 
            init_pos, 
            init_orientation, 
            config.sensors, 
            config.graphical_sensors, 
            config.graphs, 
            config.backends
        )

        # 2. Store configuration parameters
        self._config = config
        
        # Propeller configuration
        self._prop_max_thrust = config.prop_max_thrust
        self._prop_max_rpm = config.prop_max_rpm
        self._prop_thrust_coef = config.prop_thrust_coefficient
        self._prop_rotation_dir = config.prop_rotation_dir
        
        # Aerodynamic coefficients
        self._CL_0 = config.CL_0
        self._CL_alpha = config.CL_alpha
        self._CL_max = config.CL_max
        self._CL_min = config.CL_min
        
        self._CD_0 = config.CD_0
        self._CD_alpha = config.CD_alpha
        self._CD_alpha2 = config.CD_alpha2
        
        self._CY_beta = config.CY_beta
        
        self._Cm_0 = config.Cm_0
        self._Cm_alpha = config.Cm_alpha
        
        self._Cl_beta = config.Cl_beta
        self._Cn_beta = config.Cn_beta
        
        # Control surface derivatives
        self._CL_elevator = config.CL_elevator
        self._Cm_elevator = config.Cm_elevator
        self._Cl_aileron = config.Cl_aileron
        self._Cn_rudder = config.Cn_rudder
        self._CY_rudder = config.CY_rudder
        
        # Aircraft geometry
        self._wing_area = config.wing_area
        self._wing_span = config.wing_span
        self._chord = config.chord
        self._air_density = config.air_density
        
        # Drag model
        self._drag = config.drag
        
        # Current control inputs (will be updated from backend)
        self._throttle = 0.0      # 0.0 to 1.0
        self._elevator = 0.0      # -1.0 to 1.0 (radians or normalized)
        self._aileron = 0.0       # -1.0 to 1.0
        self._rudder = 0.0        # -1.0 to 1.0

        self.force_ui = ForceControlWindow()
        self.debug_drawer = DebugVisualizer()

        # Initialize CSV logging
        self._log_file_path = "forces_log.csv"
        try:
            self._log_file = open(self._log_file_path, 'w', newline='')
            self._csv_writer = csv.writer(self._log_file)
            # Write header
            self._csv_writer.writerow(['timestamp', 'thrust', 'lift', 'drag', 'side', 'mx', 'my', 'mz', 'u', 'v', 'w'])
            self._log_file.flush()
            carb.log_info(f"Initialized forces logging to {self._log_file_path}")
        except Exception as e:
            carb.log_error(f"Failed to initialize forces logging: {e}")
            self._log_file = None
            self._csv_writer = None

    def start(self):
        """
        Called when simulation starts
        """
        pass

    def stop(self):
        """
        Called when simulation stops
        """
        pass

    def update(self, dt: float):
        # ... backend updates ...
        self._update_control_inputs()

        # 2. Get real-time values from the GUI
        # forces = [x, y, z], torques = [roll, pitch, yaw]
        pos = self._state.position
        forces, torques = self.force_ui.get_inputs()

        # 3. Visualize
        # Clear previous frame's lines so they don't smear
        self.debug_drawer.clear()
        
        # Draw Force Vector (Red) - Scaled down so it fits in view
        self.debug_drawer.draw_vector(pos, forces, color=(1, 0, 0, 1), scale=0.1)
        
        # Draw Torque Vector (Blue) - Offset slightly so it doesn't overlap
        offset_pos = [pos[0], pos[1], pos[2] + 0.2]
        self.debug_drawer.draw_vector(offset_pos, torques, color=(0, 0, 1, 1), scale=0.2)

        # 3. Apply them to your object
        # Note: Ensure your backend supports these list formats
        self.apply_force(forces)
        self.apply_torque(torques)

        for backend in self._backends:
            backend.update(dt)

    def update2(self, dt: float):
        """
        Main update loop - computes and applies aerodynamic forces and moments
        This is called at every physics step.

        Args:
            dt (float): Time step in seconds
        """
        if not self._prim.IsValid():
            return
        
        # 1. Get control inputs from backend
        self._update_control_inputs()
        
        # 2. Calculate propeller thrust
        thrust_force = self._calculate_propeller_thrust()
        carb.log_info(f"Calculated Thrust: {thrust_force}")
        
        # 3. Calculate aerodynamic forces and moments
        aero_forces, aero_moments = self._calculate_aerodynamics()

        carb.log_info(f"Calculated Aero Force: {aero_forces}")
        carb.log_info(f"Calculated Aero Moment: {aero_moments}")

        # Log to CSV
        if self._csv_writer:
            try:
                # Use simulation time if available, otherwise systematic time
                timestamp = self._state.time if hasattr(self._state, 'time') else time.time()
                
                self._csv_writer.writerow([
                    timestamp,
                    thrust_force,
                    aero_forces[2], # Fz (Lift-ish)
                    aero_forces[1], # Fy (Drag-ish)
                    aero_forces[0], # Fx (Side)
                    aero_moments[0], # Mx
                    aero_moments[1], # My
                    aero_moments[2], # Mz
                    self._state.linear_body_velocity[0],
                    self._state.linear_body_velocity[1],
                    self._state.linear_body_velocity[2],
                ])
                self._log_file.flush()
            except Exception as e:
                carb.log_warn(f"Failed to log forces: {e}")
        
        # 4. Apply propeller thrust (in body frame, pointing forward)
        # self.apply_force([0.0, -thrust_force, 0.0], body_part="/body")
        self.apply_force([thrust_force, 0.0, 0.0], body_part="/body")
        
        # # 5. Apply aerodynamic forces
        self.apply_force(aero_forces, body_part="/body")
        
        # # # 6. Apply aerodynamic moments
        self.apply_torque(aero_moments, body_part="/body")
        
        # # # 7. Apply drag
        drag_force = self._drag.update(self._state, dt)
        self.apply_force(drag_force, body_part="/body")
        
        # 8. Update propeller visual (if you have a revolute joint named "propeller" or "joint0")
        # self._update_propeller_visual()
        
        # 9. Update backends
        for backend in self._backends:
            backend.update(dt)
            

    def _update_control_inputs(self):
        if len(self._backends) != 0:
            raw_inputs = self._backends[0].input_reference()
            tc = self._backends[0]._rotor_data

            def _recover_raw_cmd(value, channel_idx):
                """Reverse ThrusterControl scaling to get raw [0,1] PWM fraction."""
                scaling = tc.input_scaling[channel_idx]
                if scaling == 0:
                    scaling = 1.0
                return (value - tc.zero_position_armed[channel_idx]) / scaling - tc.input_offset[channel_idx]

            # Recover raw [0,1] fractions for each channel
            ail_raw = _recover_raw_cmd(raw_inputs[0], 0)
            ele_raw = _recover_raw_cmd(raw_inputs[1], 1)
            thr_raw = _recover_raw_cmd(raw_inputs[2], 2)
            rud_raw = _recover_raw_cmd(raw_inputs[3], 3)

            # Surface channels: center at 0.5 → remap to [-1, 1]
            # Throttle channel: already [0, 1]
            self._aileron  = np.clip((ail_raw - 0.5) * 2.0, -1.0, 1.0)
            self._elevator = np.clip((ele_raw - 0.5) * 2.0, -1.0, 1.0)
            self._throttle = np.clip(thr_raw, 0.0, 1.0)
            self._rudder   = np.clip((rud_raw - 0.5) * 2.0, -1.0, 1.0)

            carb.log_info(
                f"\n\n\n\n\n\nFixedWing Norm Inputs -> Ail: {self._aileron:.2f}, "
                f"Ele: {self._elevator:.2f}, "
                f"Thr: {self._throttle:.2f}, "
                f"Rud: {self._rudder:.2f}\n\n\n\n\n\n"
            )

        else:
            carb.log_warn("No backend detected @ fixedwing. Control inputs set to 0.")
            self._throttle = 0.0
            self._elevator = 0.0
            self._aileron = 0.0
            self._rudder = 0.0

    def _calculate_propeller_thrust(self) -> float:
        """
        Calculates thrust force from propeller based on throttle input
        
        Returns:
            float: Thrust force in Newtons (body frame X-axis)
        """
        # RPM from throttle
        rpm = self._throttle * self._prop_max_rpm
        
        # Thrust = coefficient * RPM^2
        thrust = self._prop_thrust_coef * (rpm ** 2)
        
        # Limit to maximum thrust
        thrust = np.clip(thrust, 0.0, self._prop_max_thrust)
        
        return thrust

    def _calculate_aerodynamics(self):
        """
        Calculates aerodynamic forces and moments based on current flight state
        
        Returns:
            tuple: (forces, moments) in body frame
                forces: [Fx, Fy, Fz] in Newtons
                moments: [Mx, My, Mz] in Newton-meters
        """
        
        # Get airspeed in body frame [u, v, w]
        V_body = self._state.linear_body_velocity
        u, v, w = V_body[0], V_body[1], V_body[2]
        
        # Total airspeed
        V = np.sqrt(u*u + v*v + w*w)

        carb.log_info(f"\n\n--- Aerodynamics Cycle Start ---")
        carb.log_info(f"Body Vel: {V_body}")
        carb.log_info(f"Aero Vel: [u={u:.4f}, v={v:.4f}, w={w:.4f}], Total V={V:.4f}")
        
        # Avoid division by zero
        if V < 0.1:
            carb.log_info("Speed too low (< 0.1), returning zero forces/moments.")
            return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
        
        # Calculate angle of attack (alpha) and sideslip (beta)
        alpha = np.arctan2(w, u)  # radians
        beta = np.arcsin(np.clip(v / V, -1.0, 1.0))  # radians

        carb.log_info(f"Angles: Alpha={np.degrees(alpha):.4f} deg ({alpha:.4f} rad), "
                    f"Beta={np.degrees(beta):.4f} deg ({beta:.4f} rad)")
        
        # Dynamic pressure
        q = 0.5 * self._air_density * V**2
        carb.log_info(f"Dynamic Pressure (q): {q:.4f} Pa (rho={self._air_density})")
        
        # ==========================================
        # LIFT COEFFICIENT
        # ==========================================
        CL_raw = self._CL_0 + self._CL_alpha * alpha + self._CL_elevator * self._elevator
        CL = np.clip(CL_raw, self._CL_min, self._CL_max)  # Stall limits
        carb.log_info(f"Coeff Lift (CL): {CL:.4f} (Raw: {CL_raw:.4f}, Control: {self._elevator})")
        
        # ==========================================
        # DRAG COEFFICIENT
        # ==========================================
        CD = self._CD_0 + self._CD_alpha * abs(alpha) + self._CD_alpha2 * alpha**2
        carb.log_info(f"Coeff Drag (CD): {CD:.4f}")
        
        # ==========================================
        # SIDE FORCE COEFFICIENT
        # ==========================================
        CY = self._CY_beta * beta + self._CY_rudder * self._rudder
        carb.log_info(f"Coeff Side (CY): {CY:.4f} (Control: {self._rudder})")
        
        # ==========================================
        # MOMENT COEFFICIENTS
        # ==========================================
        # Roll moment (around X-axis)
        Cl = self._Cl_beta * beta + self._Cl_aileron * self._aileron
        
        # Pitch moment (around Y-axis)
        Cm = self._Cm_0 + self._Cm_alpha * alpha + self._Cm_elevator * self._elevator
        
        # Yaw moment (around Z-axis)
        Cn = self._Cn_beta * beta + self._Cn_rudder * self._rudder

        carb.log_info(f"Moment Coeffs: Cl (Roll)={Cl:.4f}, Cm (Pitch)={Cm:.4f}, Cn (Yaw)={Cn:.4f}")
        
        # ==========================================
        # FORCES (Stability frame -> Aero Body frame)
        # ==========================================
        # In stability frame:
        # -D (drag) along velocity
        # Y (side force) perpendicular to velocity and lift
        # -L (lift) perpendicular to velocity and side force
        
        L = CL * q * self._wing_area
        D = CD * q * self._wing_area
        Y = CY * q * self._wing_area
        
        carb.log_info(f"Stability Forces: Lift={L:.4f} N, Drag={D:.4f} N, Side={Y:.4f} N")

        # Transform from stability to Aero Body frame
        # Stability frame is rotated by alpha around Y-axis relative to Aero Body
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        Fx_a = -D * cos_alpha + L * sin_alpha
        Fy_a = Y
        Fz_a = -D * sin_alpha - L * cos_alpha
        
        forces_aero = np.array([Fx_a, Fy_a, Fz_a])
        carb.log_info(f"Aero Body Forces: Fx={Fx_a:.4f}, Fy={Fy_a:.4f}, Fz={Fz_a:.4f}")
        
        # ==========================================
        # MOMENTS
        # ==========================================
        Mx_a = Cl * q * self._wing_area * self._wing_span
        My_a = Cm * q * self._wing_area * self._chord
        Mz_a = Cn * q * self._wing_area * self._wing_span
        
        moments_aero = np.array([Mx_a, My_a, Mz_a])
        carb.log_info(f"Aero Body Moments: Mx={Mx_a:.4f}, My={My_a:.4f}, Mz={Mz_a:.4f}")

        carb.log_info(f"--- Aerodynamics Cycle Ends ---\n\n")
        
        # Transform from Aero Body (FRD: X=Forward, Y=Right, Z=Down)
        # to Isaac Sim Body (FLU: X=Forward, Y=Left, Z=Up)
        # This is a 180° rotation about the X-axis (Forward axis):
        #   FLU_x =  FRD_x   (Forward = Forward)
        #   FLU_y = -FRD_y   (Left    = -Right)
        #   FLU_z = -FRD_z   (Up      = -Down)
        forces_flu = np.array([
             forces_aero[0],   # Fx (Forward) unchanged
            -forces_aero[1],   # Fy: Right → Left
            -forces_aero[2]    # Fz: Down → Up
        ])
        moments_flu = np.array([
             moments_aero[0],  # Mx (Roll) unchanged
            -moments_aero[1],  # My (Pitch) sign flipped
            -moments_aero[2]   # Mz (Yaw) sign flipped
        ])

        return forces_flu, moments_flu

    def _update_propeller_visual(self):
        """
        Updates the propeller joint velocity for visual animation
        Pervane görselini döndürmek için joint hızını ayarlar
        
        USD dosyanızda pervanenin bağlı olduğu joint'in adını bulup buraya yazmanız gerekecek
        Örnek: "propeller_joint", "joint0", "prop_joint" vb.
        """
        pass

    def set_control_inputs(self, throttle=None, elevator=None, aileron=None, rudder=None):
        """
        Manuel olarak kontrol girişlerini ayarlamak için yardımcı fonksiyon
        (Test amaçlı kullanılabilir)
        
        Args:
            throttle (float): 0.0 to 1.0
            elevator (float): -1.0 to 1.0
            aileron (float): -1.0 to 1.0
            rudder (float): -1.0 to 1.0
        """
        if throttle is not None:
            self._throttle = np.clip(throttle, 0.0, 1.0)
        if elevator is not None:
            self._elevator = np.clip(elevator, -1.0, 1.0)
        if aileron is not None:
            self._aileron = np.clip(aileron, -1.0, 1.0)
        if rudder is not None:
            self._rudder = np.clip(rudder, -1.0, 1.0)

    def get_aerodynamic_state(self):
        """
        Returns current aerodynamic state information
        Debugging ve analiz için kullanışlı
        
        Returns:
            dict: Dictionary containing aerodynamic parameters
        """
        V_body = self._state.linear_body_velocity
        u, v, w = V_body[0], V_body[1], V_body[2]
        V = np.linalg.norm(V_body)
        
        alpha = np.arctan2(w, u) if V > 0.1 else 0.0
        beta = np.arcsin(np.clip(v / V, -1.0, 1.0)) if V > 0.1 else 0.0
        
        return {
            'airspeed': V,
            'angle_of_attack': np.degrees(alpha),
            'sideslip': np.degrees(beta),
            'throttle': self._throttle,
            'elevator': self._elevator,
            'aileron': self._aileron,
            'rudder': self._rudder
        }

    def __del__(self):
        """
        Destructor - closes the log file
        """
        try:
            if hasattr(self, '_log_file') and self._log_file:
                self._log_file.close()
                carb.log_info("Closed forces log file.")
        except:
            pass

