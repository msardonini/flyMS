from dataclasses import dataclass, field, asdict
from typing import List

@dataclass
class MavlinkInterface:
    enable: bool = True
    serial_device: str = ''

@dataclass
class ImuParams:
    enable_barometer: bool = True
    R_imu_body: list = field(default_factory=list)

@dataclass
class Controller:
    pid_LPF_const_sec: float = 0.3
    roll_PID_outer: List[float] = field(default_factory=lambda: [0., 0., 0.])
    roll_PID_inner: List[float] = field(default_factory=lambda: [0., 0., 0.])
    pitch_PID_outer: List[float] = field(default_factory=lambda: [0., 0., 0.])
    pitch_PID_inner: List[float] = field(default_factory=lambda: [0., 0., 0.])
    yaw_PID: List[float] = field(default_factory=lambda: [0., 0., 0.])
    max_control_effort: List[float] = field(default_factory=lambda: [0., 0., 0.])

@dataclass
class Setpoint:
    headless_mode: bool = False
    max_setpoints_stabilized: List[float] = field(default_factory=lambda: [0., 0., 0.])
    max_setpoints_acro: List[float] = field(default_factory=lambda: [0., 0., 0.])
    throttle_limits: List[float] = field(default_factory=lambda: [0., 0., 0.])

@dataclass
class PositionController:
    pid_coeffs_x_outer: List[float] = field(default_factory=lambda: [0., 0., 0.])
    pid_coeffs_x_inner: List[float] = field(default_factory=lambda: [0., 0., 0.])
    pid_coeffs_y_outer: List[float] = field(default_factory=lambda: [0., 0., 0.])
    pid_coeffs_y_inner: List[float] = field(default_factory=lambda: [0., 0., 0.])
    pid_coeffs_z_outer: List[float] = field(default_factory=lambda: [0., 0., 0.])
    pid_coeffs_z_inner: List[float] = field(default_factory=lambda: [0., 0., 0.])
    RPY_saturation_limits: List[float] = field(default_factory=lambda: [0., 0., 0.])

@dataclass
class Filters:
    # elliptic filter 10th order 0.25 dB passband ripple 80 dB min Cutoff 0.8 cutoff frq
    imu_lpf_num: List[float] = field(default_factory=lambda: [0., 0., 0.])
    imu_lpf_den: List[float] = field(default_factory=lambda: [0., 0., 0.])

@dataclass
class flyMSConfig:
    log_filepath: str = ''
    debug_mode: bool = False
    enable_gps: bool = False
    flight_mode: int = 1
    enable_logging: bool = True

    mavlink_interface: MavlinkInterface = MavlinkInterface()
    imu_params: ImuParams = ImuParams()
    controller: Controller = Controller()
    setpoint: Setpoint = Setpoint()
    position_controller: PositionController = PositionController()
    filters: Filters = Filters()
