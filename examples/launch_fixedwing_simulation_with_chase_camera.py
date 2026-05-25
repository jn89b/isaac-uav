#!/usr/bin/env python
"""
| File: launch_fixedwing_simulation_with_chase_camera.py
| Author: [Your Name]
| License: BSD-3-Clause
| Description:
|   Isaac Sim standalone app for fixed-wing aircraft simulation with
|   ArduPilot/PX4 backend and an automatic chase camera.
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
import omni.usd
import omni.kit.viewport.utility
from omni.isaac.core.world import World

from pxr import Gf, UsdGeom

# Import the Pegasus API for simulating vehicles
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS, ROBOTS
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (
    ArduPilotMavlinkBackend,
    ArduPilotMavlinkBackendConfig,
)

# from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation

# Import the FixedWing class
from pegasus.simulator.logic.vehicles.fixedwing import FixedWing, FixedWingConfig


class FixedWingApp:
    def __init__(self):
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World - controls physics, spawning assets, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load simulation environment
        # Options: "Curved Gridroom", "Default Environment", "Black Gridroom",
        # "Hospital", "Office", "Warehouse"
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])

        # Create the fixed-wing aircraft
        self.create_fixedwing_vehicle()

        # Aircraft base color in RGB [0.0, 1.0]
        self.aircraft_color_rgb = (0.95, 0.12, 0.12)
        self.apply_aircraft_color(self.aircraft_color_rgb)

        # Create the follow/chase camera
        self.create_chase_camera()

        # Reset the simulation environment so that all articulations are initialized
        self.world.reset()

        # Auxiliary variable for the timeline callback
        self.stop_sim = False

        print("✓ Fixed-wing simulation initialized successfully!")

    def create_fixedwing_vehicle(self):
        """
        Create a single fixed-wing aircraft with configured backend.
        """
        config = FixedWingConfig()

        # Propeller/Motor settings
        config.prop_max_thrust = 100.0
        config.prop_max_rpm = 10000.0
        config.prop_thrust_coefficient = 0.000075
        config.prop_rotation_dir = 1  # 1: CCW, -1: CW

        # Aircraft geometry
        config.wing_area = 2.36
        config.wing_span = 4.46
        config.chord = 0.53

        # Aerodynamic coefficients
        config.CL_0 = 0.3
        config.CL_alpha = 4.0
        config.CL_max = 1.5
        config.CD_0 = 0.025

        # Control surface effectiveness
        config.Cm_elevator = -1.5
        config.Cl_aileron = 0.3
        config.Cn_rudder = -0.05

        # Simulation modes:
        # manual      : User provides forces. Aerodynamics are NOT calculated.
        # thrust_only : User provides thrust. Aerodynamics are calculated.
        # autonomous  : Backend controls aircraft. Aerodynamics are calculated.
        config.simulation_mode = "autonomous"
        # config.debug_mode = True

        ardupilot_config = ArduPilotMavlinkBackendConfig(
            {
                "vehicle_id": 0,

                # Use 0.0.0.0 so Pegasus listens on all Windows interfaces,
                # including the WSL2 virtual adapter.
                "connection_type": "udpin",
                "connection_ip": "0.0.0.0",
                "connection_baseport": 14550,

                # Keep False when ArduPilot is launched manually from WSL2.
                "ardupilot_autolaunch": False,
                "ardupilot_dir": "",

                # Fixed-wing / ArduPlane settings.
                "ardupilot_vehicle_model": "plane",
                "ardupilot_vehicle": "ArduPlane",
            }
        )

        # Combine backends
        config.backends = [
            ArduPilotMavlinkBackend(config=ardupilot_config),
        ]

        self.aircraft_path = "/World/fixedwing0"

        self.aircraft = FixedWing(
            stage_prefix=self.aircraft_path,
            usd_file=ROBOTS["Fixed Wing"],
            vehicle_id=0,
            init_pos=[0.0, 0.0, 1.0],
            init_orientation=Rotation.from_euler(
                "XYZ", [0.0, 0.0, 0.0], degrees=True
            ).as_quat(),
            config=config,
        )

        print("✓ Fixed-wing aircraft created.")

    def apply_aircraft_color(self, rgb):
        """
        Apply a displayColor tint to every mesh under the aircraft root prim.
        """
        stage = omni.usd.get_context().get_stage()
        aircraft_root = stage.GetPrimAtPath(self.aircraft_path)

        if not aircraft_root.IsValid():
            carb.log_warn(f"Aircraft prim not found at {self.aircraft_path}; skipping color update")
            return

        color = Gf.Vec3f(
            max(0.0, min(1.0, float(rgb[0]))),
            max(0.0, min(1.0, float(rgb[1]))),
            max(0.0, min(1.0, float(rgb[2]))),
        )

        stack = [aircraft_root]
        tinted_meshes = 0

        while stack:
            prim = stack.pop()
            stack.extend(prim.GetChildren())

            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                display_color = mesh.GetDisplayColorPrimvar()
                if not display_color:
                    display_color = mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
                display_color.Set([color])
                tinted_meshes += 1

        carb.log_info(f"Applied aircraft color {tuple(color)} to {tinted_meshes} mesh prim(s)")

    def create_chase_camera(self):
        """
        Create a camera that will be updated every frame to follow the aircraft.

        Tuning variables:
            self.chase_distance_m : distance behind the aircraft
            self.chase_height_m   : height above the aircraft
            self.chase_side_m     : side offset, useful for over-shoulder view
            self.camera_smoothing : 0.0 = no movement, 1.0 = snap instantly
        """
        self.camera_path = "/World/ChaseCamera"

        # Camera tuning
        self.chase_distance_m = 18.0
        self.chase_height_m = 6.0
        self.chase_side_m = 0.0
        self.camera_smoothing = 0.12

        # Internal state for smoothing
        self._camera_initialized = False
        self._smoothed_camera_pos = None
        self._camera_xform_op = None

        stage = omni.usd.get_context().get_stage()
        camera = UsdGeom.Camera.Define(stage, self.camera_path)

        # Reasonable chase-camera lens settings
        camera.GetFocalLengthAttr().Set(24.0)
        camera.GetHorizontalApertureAttr().Set(20.955)

        # Keep a persistent transform op so we only update its value each frame.
        camera_xform = UsdGeom.Xformable(camera.GetPrim())
        self._camera_xform_op = camera_xform.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)

        # Switch active viewport to the chase camera
        viewport = omni.kit.viewport.utility.get_active_viewport()
        if viewport is not None:
            viewport.camera_path = self.camera_path

        print("✓ Chase camera created.")

    @staticmethod
    def _safe_normalize(v: Gf.Vec3d, fallback: Gf.Vec3d) -> Gf.Vec3d:
        """
        Normalize a vector while avoiding divide-by-zero issues.
        """
        length = v.GetLength()
        if length < 1e-6:
            return fallback
        return v / length

    def _compute_look_at_matrix(
        self,
        eye: Gf.Vec3d,
        target: Gf.Vec3d,
        up_hint: Gf.Vec3d = Gf.Vec3d(0.0, 0.0, 1.0),
    ) -> Gf.Matrix4d:
        """
        Build a USD camera world transform that places the camera at `eye`
        and aims it at `target`.

        USD cameras look along local -Z with local +Y as up.
        """
        forward = self._safe_normalize(target - eye, Gf.Vec3d(1.0, 0.0, 0.0))

        # Avoid a degenerate cross product if looking almost straight up/down.
        if abs(Gf.Dot(forward, up_hint)) > 0.98:
            up_hint = Gf.Vec3d(0.0, 1.0, 0.0)

        right = self._safe_normalize(Gf.Cross(forward, up_hint), Gf.Vec3d(0.0, 1.0, 0.0))
        true_up = self._safe_normalize(Gf.Cross(right, forward), Gf.Vec3d(0.0, 0.0, 1.0))

        # Camera local axes in world coordinates:
        # +X = right, +Y = up, +Z = backward, so -Z looks forward.
        backward = -forward

        transform = Gf.Matrix4d(
            right[0],    right[1],    right[2],    0.0,
            true_up[0],  true_up[1],  true_up[2],  0.0,
            backward[0], backward[1], backward[2], 0.0,
            eye[0],      eye[1],      eye[2],      1.0,
        )

        return transform

    def update_chase_camera(self):
        """
        Update the chase camera position and orientation so it follows the aircraft.

        The camera offset is computed in the aircraft's local/world orientation:
        - behind along the aircraft local -X axis
        - side along the aircraft local +Y axis
        - above along world +Z
        """
        stage = omni.usd.get_context().get_stage()

        aircraft_prim = stage.GetPrimAtPath(self.aircraft_path)
        camera_prim = stage.GetPrimAtPath(self.camera_path)

        if not aircraft_prim.IsValid() or not camera_prim.IsValid():
            return

        aircraft_xform = UsdGeom.Xformable(aircraft_prim)
        aircraft_world = aircraft_xform.ComputeLocalToWorldTransform(0.0)
        aircraft_pos = aircraft_world.ExtractTranslation()

        # Extract aircraft local axes from its world rotation.
        # This is more robust than reading matrix rows directly.
        rot = aircraft_world.ExtractRotationQuat()

        local_x = self._safe_normalize(
            rot.Transform(Gf.Vec3d(1.0, 0.0, 0.0)),
            Gf.Vec3d(1.0, 0.0, 0.0),
        )
        local_y = self._safe_normalize(
            rot.Transform(Gf.Vec3d(0.0, 1.0, 0.0)),
            Gf.Vec3d(0.0, 1.0, 0.0),
        )

        desired_camera_pos = (
            aircraft_pos
            - local_x * self.chase_distance_m
            + local_y * self.chase_side_m
            + Gf.Vec3d(0.0, 0.0, self.chase_height_m)
        )

        # Aim slightly ahead of the aircraft so the view points in the direction of motion.
        look_target = aircraft_pos + local_x * 6.0

        # Smooth camera movement to avoid jitter.
        if not self._camera_initialized or self._smoothed_camera_pos is None:
            self._smoothed_camera_pos = desired_camera_pos
            self._camera_initialized = True
        else:
            alpha = max(0.0, min(1.0, self.camera_smoothing))
            self._smoothed_camera_pos = (
                self._smoothed_camera_pos * (1.0 - alpha)
                + desired_camera_pos * alpha
            )

        transform = self._compute_look_at_matrix(self._smoothed_camera_pos, look_target)

        if self._camera_xform_op is None:
            camera_xform = UsdGeom.Xformable(camera_prim)
            self._camera_xform_op = camera_xform.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)

        self._camera_xform_op.Set(transform)

    def run(self):
        """
        Main application loop - executes physics steps.
        """
        self.timeline.play()
        print("▶ Simulation started!")

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)
            self.update_chase_camera()

        # Cleanup and stop
        carb.log_warn("Fixed-wing Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    """
    Main entry point.
    """
    app = FixedWingApp()
    app.run()


if __name__ == "__main__":
    main()
