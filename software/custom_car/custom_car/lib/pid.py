### FILE: custom_car/custom_car/lib/pid.py
###
### DESCRIPTION: PID Control Implementation
###
### USAGE:
###     pid = PIDController(kp=0.005, ki=0.0, kd=0.05 ... )
###     output = pid.compute(error=42.0, dt=0.05)
###     pid.reset()


from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PIDConfig:
    """config for a PID controller"""
    kp:           float = 0.005
    ki:           float = 0.000
    kd:           float = 0.050

    output_min:   float = -1.2
    output_max:   float = 1.2

    integral_min: float = -50.0
    integral_max: float = 50.0


class PIDController:

    def __init__(self, config: Optional[PIDConfig] = None) -> None:
        """ Constructor """
        self.config     = config or PIDConfig()
        self._integral: float = 0.0
        self._prev_error: float = 0.0

    #####################################
    #########   PUBLIC METHODS  #########
    #####################################

    def compute(self, error: float, dt: float) -> float:
        """ THE method that produces a PID output from an error and a timestep (dt) """
        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt}")

        cfg = self.config

        p_term = cfg.kp * error

        self._integral += error * dt
        self._integral = _clamp(self._integral, cfg.integral_min, cfg.integral_max)
        i_term = cfg.ki * self._integral

        d_term = cfg.kd * (error - self._prev_error) / dt
        self._prev_error = error

        raw = p_term + i_term + d_term
        return _clamp(raw, cfg.output_min, cfg.output_max)

    def reset(self) -> None:
        """clear past I and D s."""
        self._integral = 0.0
        self._prev_error = 0.0

    def update_config(self, config: PIDConfig) -> None:
        """swap gains whenever we need"""
        self.config = config

    #################################
    #########   PROPERTIES  #########
    #################################


    @property
    def integral(self) -> float:
        return self._integral

    @property
    def prev_error(self) -> float:
        return self._prev_error


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))