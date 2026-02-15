"""
Autonomous Mission State Machine

States:
  IDLE -> SCAN_CUBE -> APPROACH -> READ_QR -> PICK_UP -> STORE -> CHECK_DONE -> COMPLETE
                 ^                                              |
                 |______________________________________________|  (more cubes)
"""

import logging
import time
from enum import Enum, auto

logger = logging.getLogger(__name__)


class State(Enum):
    IDLE = auto()
    SCAN_CUBE = auto()
    APPROACH = auto()
    READ_QR = auto()
    PICK_UP = auto()
    STORE = auto()
    CHECK_DONE = auto()
    COMPLETE = auto()
    ERROR = auto()


class MissionStateMachine:
    """Finite state machine for autonomous pick-and-place mission."""

    def __init__(self, perception, base_bridge, arm_bridge):
        """
        Args:
            perception: Object with detect_qr() and detect_color() methods.
            base_bridge: SerialBridge to base ESP32.
            arm_bridge: SerialBridge to arm ESP32.
        """
        self.perception = perception
        self.base = base_bridge
        self.arm = arm_bridge

        self.state = State.IDLE
        self.cubes_picked = 0
        self.max_cubes = 3
        self.current_cube_color = None
        self.state_start_time = 0.0
        self.state_timeout_s = 30.0  # Per-state timeout

        # Results tracking
        self.picked_colors = []

    def start(self):
        """Begin autonomous mission."""
        logger.info("Starting autonomous mission")
        self._transition(State.SCAN_CUBE)

    def stop(self):
        """Abort mission and return to IDLE."""
        logger.info("Mission stopped")
        self._stop_all()
        self._transition(State.IDLE)

    def update(self):
        """
        Call this in the main loop. Executes the current state logic.

        Returns:
            Current state.
        """
        # Check state timeout
        if self.state not in (State.IDLE, State.COMPLETE, State.ERROR):
            elapsed = time.time() - self.state_start_time
            if elapsed > self.state_timeout_s:
                logger.warning(f"State {self.state.name} timed out after {elapsed:.1f}s")
                self._transition(State.ERROR)
                return self.state

        if self.state == State.IDLE:
            pass  # Waiting for start()

        elif self.state == State.SCAN_CUBE:
            self._state_scan_cube()

        elif self.state == State.APPROACH:
            self._state_approach()

        elif self.state == State.READ_QR:
            self._state_read_qr()

        elif self.state == State.PICK_UP:
            self._state_pick_up()

        elif self.state == State.STORE:
            self._state_store()

        elif self.state == State.CHECK_DONE:
            self._state_check_done()

        elif self.state == State.COMPLETE:
            pass  # Mission done

        elif self.state == State.ERROR:
            self._stop_all()

        return self.state

    def _state_scan_cube(self):
        """Scan for cubes using the camera."""
        # TODO: Use perception to find cube in camera frame
        # For now, placeholder that transitions after finding a cube
        detection = self.perception.detect_qr()
        if detection:
            self.current_cube_color = detection['color']
            logger.info(f"Cube spotted: {self.current_cube_color}")
            self._transition(State.APPROACH)

    def _state_approach(self):
        """Move base to position arm within reach of the cube."""
        # TODO: Use perception feedback to guide approach
        # Send velocity commands to base to align with cube
        # Transition when close enough
        pass

    def _state_read_qr(self):
        """Read QR code at close range to confirm color."""
        detection = self.perception.detect_qr()
        if detection:
            self.current_cube_color = detection['color']
            logger.info(f"QR confirmed: {self.current_cube_color}")
            self._transition(State.PICK_UP)

    def _state_pick_up(self):
        """Command arm to pick up the cube."""
        # TODO: Send arm Cartesian command to pedestal height,
        # close gripper, verify grasp
        # Transition when grasp confirmed
        pass

    def _state_store(self):
        """Move cube to on-board storage bin."""
        # TODO: Move arm to bin position based on color, open gripper
        # Transition when stored
        self.cubes_picked += 1
        self.picked_colors.append(self.current_cube_color)
        logger.info(f"Cube stored: {self.current_cube_color} "
                     f"({self.cubes_picked}/{self.max_cubes})")
        self._transition(State.CHECK_DONE)

    def _state_check_done(self):
        """Check if more cubes need to be picked."""
        if self.cubes_picked >= self.max_cubes:
            logger.info("All cubes collected!")
            self._transition(State.COMPLETE)
        else:
            self._transition(State.SCAN_CUBE)

    def _transition(self, new_state: State):
        """Transition to a new state."""
        logger.info(f"State: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_start_time = time.time()

    def _stop_all(self):
        """Stop all actuators."""
        from communication.protocol import encode_velocity, encode_gripper
        self.base.send(encode_velocity(0.0, 0.0, 0.0))
