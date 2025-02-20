import time
from dataclasses import replace
import torch

import trossen_slate as slate
import threading
from lerobot.common.robot_devices.robots.configs import AlohaMobileRobotConfig

class AlohaMobileRobot():

    def __init__(self, config: AlohaMobileRobotConfig | None = None, **kwargs):
        super().__init__()
        if config is None:
            self.config = AlohaMobileRobotConfig(**kwargs)
        else:
            # Overwrite config arguments using kwargs
            self.config = replace(config, **kwargs)
        self.robot_type = self.config.type
        self.cameras = self.config.cameras
        self.leader_arms= []
        self.follower_arms = []
        self.is_connected = False
        self.teleop = None
        self.logs = {}

        self.base = slate.TrossenSlate()
        self.base.init_base()

        self.read_data = False

        self.state_keys = None
        self.action_keys = None

        self.log_data = slate.ChassisData()
        self.update_thread = None

    @property
    def camera_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            key = f"observation.images.{cam_key}"
            cam_ft[key] = {
                "shape": (cam.height, cam.width, cam.channels),
                "names": ["height", "width", "channels"],
                "info": None,
            }
        return cam_ft

    @property
    def motor_features(self) -> dict:
        action_names = ['linear_vel', 'angular_vel']
        state_names = ['odom_x', 'odom_y', 'odom_theta', 'linear_vel', 'angular_vel']
        return {
            "action": {
                "dtype": "float32",
                "shape": (len(action_names),),
                "names": action_names,
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(state_names),),
                "names": state_names,
            },
        }

    @property
    def features(self):
        return {**self.motor_features, **self.camera_features}

    @property
    def has_camera(self):
        return len(self.cameras) > 0

    @property
    def num_cameras(self):
        return len(self.cameras)

    @property
    def available_arms(self):
        available_arms = []
        return available_arms

    def connect(self) -> None:
        self.base.init_base()
        self.is_connected = True

        # Start Reading
        self.read_data = True
        self.update_thread = threading.Thread(target=self.update_values)
        self.update_thread.start()

        # Add cameras connection here
        # Add Arm Connection Here

    def update_values(self):
        while self.read_data:
            self.base.update_state()
            self.base.read(self.log_data)

    def get_state(self) -> dict:
        return {
            "odom_x": self.log_data.odom_x,
            "odom_y": self.log_data.odom_y,
            "odom_theta": self.log_data.odom_z,
            "linear_vel": self.log_data.vel_x,
            "angular_vel": self.log_data.vel_z,
        }

    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:

        if not self.is_connected:
            raise ConnectionError()

        if self.teleop is None:
            pass

        before_read_t = time.perf_counter()
        state = self.get_state()
        action = [state['linear_vel'], state['angular_vel']]

        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        before_write_t = time.perf_counter()
        pass
        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t

        if not record_data:
            return
        state = torch.as_tensor(list(state.values()))
        action = torch.as_tensor(list(action))

        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = state
        action_dict["action"] = action

        return obs_dict, action_dict

    def capture_observation(self):

        before_read_t = time.perf_counter()
        state = self.get_state()
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        state = torch.as_tensor(list(state.values()))

        obs_dict = {}
        obs_dict["observation.state"] = state

        return obs_dict

    def send_action(self, action):
        linear_vel, angular_vel = action.tolist()
        before_write_t = time.perf_counter()
        self.base.set_cmd_vel(linear_vel, angular_vel)
        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t

        return action

    def disconnect(self):
        self.read_data = False
        if self.update_thread is not None:
            self.update_thread.join()
        self.is_connected = False

    def __del__(self):
        self.disconnect()