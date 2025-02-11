
import time
import traceback
import trossen_arm_driver as trossen
import numpy as np

from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
from lerobot.common.robot_devices.motors.configs import TrossenArmDriverConfig
import scipy.interpolate

import threading

TROSSEN_ARM_MODELS = {
    "V0_LEADER": [trossen.Model.wxai_v0, trossen.StandardEndEffector.wxai_v0_leader],
    "V0_FOLLOWER": [trossen.Model.wxai_v0, trossen.StandardEndEffector.wxai_v0_follower],
}

class TrossenArmDriver:
    """
        The `TrossenArmDriver` class provides an interface for controlling 
        Trossen Robotics' robotic arms. It leverages the trossen_arm_driver_py for communication with arms.

        This class allows for configuration, torque management, and motion control of robotic arms. It includes features for handling connection states, moving the 
        arm to specified poses, and logging timestamps for debugging and performance analysis.

        ### Key Features:
        - **Multi-motor Control:** Supports multiple motors connected to a bus.
        - **Mode Switching:** Enables switching between position and gravity 
        compensation modes.
        - **Motion Interpolation:** Implements smooth motion using `PchipInterpolator` for trajectory generation.
        - **Home and Sleep Pose Management:** Automatically transitions the arm to home and sleep poses for safe operation.
        - **Error Handling:** Raises specific exceptions for connection and operational errors.
        - **Logging:** Captures timestamps for operations to aid in debugging.

        ### Example Usage:
        ```python
        motors = {
            "waist": (1, "dameo"),
            "shoulder": (2, "dameo"),y
            "elbow": (4, "dameo"),
            "forearm_roll": (6, "dameo"),
            "wrist_angle": (7, "dameo"),
            "wrist_rotate": (8, "dameo"),
            "gripper": (9, "dameo"),
        }
        arm_driver = TrossenArmDriver(
            motors=motors,
            ip="192.168.0.10",
            model="V0_LEADER",
        )
        arm_driver.connect()

        # Read motor positions
        positions = arm_driver.read("Present_Position")

        # Move to a new position
        arm_driver.write("Goal_Position", positions + 30)

        # Disconnect when done
        arm_driver.disconnect()
        ```
    """


    def __init__(
        self,
        config: TrossenArmDriverConfig,
    ):
        self.ip = config.ip
        self.model = config.model
        self.mock = config.mock
        self.driver = None
        self.calibration = None
        self.is_connected = False
        self.group_readers = {}
        self.group_writers = {}
        self.logs = {}

        self.dt= 1e-3
        self.home_pose = np.array([0, np.pi/12, np.pi/12, 0, 0, 0, 0])
        self.sleep_pose = np.array([0, 0, 0, 0, 0, 0, 0])

        self.motors={
                    # name: (index, model)
                    "waist": [1, "damaeo"],
                    "shoulder": [2, "damaeo"],
                    "elbow": [3, "damaeo"],
                    "forearm_roll": [4, "damaeo"],
                    "wrist_angle": [5, "damaeo"],
                    "wrist_rotate": [6, "damaeo"],
                    "gripper": [7, "damaeo"],
                }
        
        self.target_positions = None
        self.current_positions = None
        self.running = True  # Control flag for background thread
        self.write_freq = 30  # Hz (Leader update frequency)
        self.driver_freq = 150  # Hz (Follower update frequency)
        self.interp_buffer = None  # Interpolated trajectory buffer
        self.lock = threading.Lock()  # Thread safety lock

        # Start the background update thread
        self.update_thread = threading.Thread(target=self._run_follower_motion, daemon=True)
        self.update_thread.start()

    def connect(self):
        print(f"Connecting to {self.model} arm at {self.ip}...")
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
            f"TrossenArmDriver({self.ip}) is already connected. Do not call `motors_bus.connect()` twice."
            )

        print("Initializing the drivers...")

        # Initialize the driver
        self.driver = trossen.TrossenArmDriver()

        # Get the model configuration
        try:
            model_name, model_end_effector = TROSSEN_ARM_MODELS[self.model]
        except KeyError:
            raise ValueError(f"Unsupported model: {self.model}")

        print("Configuring the drivers...")

        # Configure the driver
        try:
            self.driver.configure(model_name, model_end_effector, self.ip, True)
        except Exception:
            traceback.print_exc()
            print(
            f"Failed to configure the driver for the {self.model} arm at {self.ip}."
            )
            raise

        # Move the arms to the home pose
        self.driver.move_arm_to(2.0, self.home_pose[:6])

        # Allow to read and write
        self.is_connected = True


    def reconnect(self):
        try:
            model_name, model_end_effector = TROSSEN_ARM_MODELS[self.model]
        except KeyError:
            raise ValueError(f"Unsupported model: {self.model}")
        try:
            self.driver.configure(model_name, model_end_effector, self.ip, False)
        except Exception:
            traceback.print_exc()
            print(
            f"Failed to configure the driver for the {self.model} arm at {self.ip}."
            )
            raise

        self.is_connected = True


    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    def set_calibration(self, calibration: dict[str, list]):
        self.calibration = calibration

    def apply_calibration_autocorrect(self, values: np.ndarray | list, motor_names: list[str] | None):
        pass

    def apply_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        pass

    def autocorrect_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        pass

    def revert_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        pass


    def _interpolate_positions(self, new_target):
        """Interpolates between previous and new target positions smoothly."""
        # if not self.driver.receive_robot_output():
        #     print("Failed to receive the states!")

        self.current_positions = np.array(self.driver.get_positions())

        t = np.linspace(0, 1, int(self.driver_freq / self.write_freq))  # Time steps
        interpolator = scipy.interpolate.interp1d([0, 1], np.vstack([self.current_positions, new_target]), axis=0, kind='linear')
        return interpolator(t)

    def _run_follower_motion(self):
        """Continuously writes smoothed positions at 150Hz."""
        while self.running:
            if self.interp_buffer is not None:
                for pos in self.interp_buffer:
                    with self.lock:
                        self.driver.set_positions(pos)
                        if not self.driver.receive_robot_output():
                            print("Failed to receive the states!")
                        self.current_positions = self.driver.get_positions()

                    time.sleep(1 / self.driver_freq)  # Maintain 150Hz write rate

    
    def stop(self):
        """Stops the motion thread."""
        self.running = False
        self.update_thread.join()

    def radians_to_discrete(self,radians, resolution=4096):
        """
        Convert radians to discrete integer values with a given resolution.
        Args:
            radians (float or np.ndarray): Input radians in the range [-π, π].
            resolution (int): Number of discrete steps (default: 4096).
        Returns:
            np.ndarray: Discrete integer values representing the radians.
        """
        radians = np.array(radians, dtype=np.float32)
        # Ensure radians are in the range [-π, π]
        if np.any((radians[:6] < -np.pi) | (radians[:6] > np.pi)):
            raise ValueError("Radians must be in the range [-π, π].")

        # Map radians to discrete integer values
        discrete_values = ((radians + np.pi) / (2 * np.pi) * (resolution - 1)).round()
        # Convert radians to degrees
        # discrete_values = np.degrees(radians)

        return discrete_values

    def discrete_to_radians(self, discrete_values, resolution=4096):
        """
        Convert discrete integer values back to radians with a given resolution.
        Args:
            discrete_values (np.ndarray): Discrete integer values.
            resolution (int): Number of discrete steps (default: 4096).
        Returns:
            np.ndarray: Restored radians values.
        """
        # Map discrete values back to radians
        radians = (discrete_values / (resolution - 1) * (2 * np.pi)) - np.pi
        # Convert degrees to radians
        # radians = discrete_values * 3.14159 / 180
        # radians = [round(r, 4) for r in radians]
        return radians


    def read(self, data_name, motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"TrossenArmMotorsBus({self.port}) is not connected. You need to run `motors_bus.connect()`."
            )

        start_time = time.perf_counter()

        # Read the present position of the motors
        if data_name == "Present_Position":

            self.driver.request_robot_output()
            # Get the positions of the motors
            values = self.driver.get_positions()
            # print("=================================================================================================")
            # print(f"Leader Values {[f'{value:.4f}' for value in values]}")
            values = self.radians_to_discrete(values)

            # Receive the updated states
            if not self.driver.receive_robot_output():
                print("Failed to receive the states!")
        else:
            values = None
            print(f"Data name: {data_name} is not supported for reading.")

        # TODO: Add support for reading other data names as required

        self.logs["delta_timestamp_s_read"] = time.perf_counter() - start_time

        values = np.array(values, dtype=np.float32)
        return values


    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"DynamixelMotorsBus({self.port}) is not connected. You need to run `motors_bus.connect()`."
            )

        start_time = time.perf_counter()

        # Write the goal position of the motors
        if data_name == "Goal_Position":

            # Convert int32 to float and scale it down
            values = self.discrete_to_radians(values)

            # print(f"Follower Write {[f'{value:.4f}' for value in values]}")
            # print("=================================================================================================")

            # Set the positions of the motors
            self.driver.set_positions(values)
            # with self.lock:
            #     self.interp_buffer = self._interpolate_positions(values)  # Generate smooth steps
            # Receive the updated states
            if not self.driver.receive_robot_output():
                print("Failed to receive the states!")

        # Enable or disable the torque of the motors
        elif data_name == "Torque_Enable":
            # Set the arms to POSITION mode
            if values == 1:
                modes = self.driver.get_modes()
                if modes != [trossen.Mode.position] * 7:
                    self.driver.set_mode(trossen.Mode.position)
                else:
                    print("Mode is already set to POSITION")
            # Set the arms to GRAVITY_COMPENSATION mode
            else:
                modes = self.driver.get_modes()
                if modes!= [trossen.Mode.torque] * 7:
                    self.driver.set_mode(trossen.Mode.torque)
                    self.driver.set_torques([0, 0, 0, 0, 0, 0, 0])
                else:
                    print("Mode is already set to GRAVITY_COMPENSATION")
        else:
            print(f"Data name: {data_name} value: {values} is not supported for writing.")

        self.logs["delta_timestamp_s_write"] = time.perf_counter() - start_time

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"Trossen Arm Driver ({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )
        print("Moving the motors to their sleep position...")
        modes = self.driver.get_modes()
        if modes!= [trossen.Mode.torque] * 7:
            self.driver.set_mode(trossen.Mode.torque)
            self.driver.set_torques([0, 0, 0, 0, 0, 0, 0])
        time.sleep(1)
        print("Modes are set to", self.driver.get_modes())
        self.driver.move_arm_to(2.0, self.sleep_pose[:6])

        print("Setting the motors to idle mode...")

        self.driver.set_mode(trossen.Mode.idle)

        print("Cleaning up the drivers...")
        self.driver.cleanup()



        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()
