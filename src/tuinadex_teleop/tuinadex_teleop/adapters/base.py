"""Abstract hardware adapter interfaces (Ports & Adapters pattern)."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

try:
    from tuinadex_teleop.core.types import TeleopCommand, RobotObservation
except ImportError:
    from tuina_teleop.core.types import TeleopCommand, RobotObservation



class BaseRobotAdapter(ABC):
    """Unified robot hardware abstraction contract (Arm 6-DOF + Hand 16-DOF).
    
    Subclasses:
    - MockSimAdapter: In-memory loopback for fast unit testing.
    - StandaloneAdapter: Direct CAN (RS motors) and serial (Dynamixel) driver.
    - TuinaDexBridgeAdapter: Publishes /arm/joint_command and /hand/joint_command to ROS2.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Establish communication with underlying bus or simulator."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Safely park and disable motors."""
        pass

    @abstractmethod
    def send_command(self, cmd: TeleopCommand) -> None:
        """Send target joint positions, velocities, and impedances."""
        pass

    @abstractmethod
    def read_observation(self) -> RobotObservation:
        """Read latest synchronized joint states, velocities, and efforts."""
        pass

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether communication is healthy."""
        pass

