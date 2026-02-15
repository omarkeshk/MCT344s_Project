"""
Master Node — Main entry point for the supervisory controller (Raspberry Pi).

Orchestrates:
  - Serial communication with Base and Arm ESP32 modules
  - Camera perception (QR detection + color classification)
  - Gamepad teleoperation
  - Autonomous mission state machine
  - Telemetry display
"""

import sys
import os
import time
import logging
import signal
import yaml

# Add src to path so imports work
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from communication.serial_bridge import SerialBridge
from communication.protocol import (
    Mode, encode_velocity, encode_mode_switch, encode_arm_joints,
    encode_gripper, encode_arm_cartesian,
)
from perception.camera_driver import CameraDriver
from perception.qr_detector import QRDetector
from perception.color_classifier import ColorClassifier
from teleop.gamepad_driver import GamepadDriver
from mission.state_machine import MissionStateMachine

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
)
logger = logging.getLogger('main')


class MasterNode:
    """Main supervisory controller."""

    def __init__(self, config_path: str):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.mode = Mode.TELEOP
        self.running = False

        # Communication
        self.base_bridge = SerialBridge(
            port=self.config['comm']['base_port'],
            baud_rate=self.config['comm']['baud_rate'],
            name="base",
        )
        self.arm_bridge = SerialBridge(
            port=self.config['comm']['arm_port'],
            baud_rate=self.config['comm']['baud_rate'],
            name="arm",
        )

        # Perception
        cam_cfg = self.config['camera']
        self.camera = CameraDriver(
            device_id=cam_cfg['device_id'],
            width=cam_cfg['width'],
            height=cam_cfg['height'],
            fps=cam_cfg['fps'],
        )
        self.qr_detector = QRDetector()
        self.color_classifier = ColorClassifier()

        # Teleop
        teleop_cfg = self.config['teleop']
        self.gamepad = GamepadDriver(deadzone=teleop_cfg['deadzone'])

        # Mission
        self.mission = MissionStateMachine(
            perception=self,  # This class provides detect_qr()
            base_bridge=self.base_bridge,
            arm_bridge=self.arm_bridge,
        )

        # Heartbeat timing
        self.last_heartbeat_time = 0.0
        self.heartbeat_interval = self.config['comm']['heartbeat_interval_s']

    def detect_qr(self):
        """Perception interface used by the state machine."""
        frame = self.camera.get_frame()
        if frame is None:
            return None
        return self.qr_detector.detect(frame)

    def start(self):
        """Initialize all subsystems."""
        logger.info("Starting Master Node...")

        try:
            self.base_bridge.open()
        except Exception as e:
            logger.error(f"Base bridge failed: {e}")

        try:
            self.arm_bridge.open()
        except Exception as e:
            logger.error(f"Arm bridge failed: {e}")

        try:
            self.camera.open()
        except Exception as e:
            logger.error(f"Camera failed: {e}")

        try:
            self.gamepad.init()
        except Exception as e:
            logger.error(f"Gamepad failed: {e}")

        self.running = True
        logger.info("Master Node started")

    def stop(self):
        """Shut down all subsystems."""
        logger.info("Stopping Master Node...")
        self.running = False

        # Send ESTOP
        estop_frame = encode_mode_switch(Mode.ESTOP)
        self.base_bridge.send(estop_frame)
        self.arm_bridge.send(estop_frame)

        self.mission.stop()
        self.camera.close()
        self.gamepad.close()
        self.base_bridge.close()
        self.arm_bridge.close()
        logger.info("Master Node stopped")

    def run(self):
        """Main loop."""
        self.start()

        try:
            while self.running:
                loop_start = time.time()

                # Send heartbeats
                now = time.time()
                if now - self.last_heartbeat_time >= self.heartbeat_interval:
                    self.base_bridge.send_heartbeat()
                    self.arm_bridge.send_heartbeat()
                    self.last_heartbeat_time = now

                if self.mode == Mode.TELEOP:
                    self._teleop_update()
                elif self.mode == Mode.AUTO:
                    self._auto_update()

                # Print telemetry
                self._print_telemetry()

                # Rate limit to ~50 Hz
                elapsed = time.time() - loop_start
                sleep_time = max(0, 0.02 - elapsed)
                time.sleep(sleep_time)

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.stop()

    def _teleop_update(self):
        """Handle manual teleoperation mode."""
        self.gamepad.update()

        # Check for mode switch
        if self.gamepad.btn_a:
            self._switch_mode(Mode.AUTO)
            return

        if self.gamepad.btn_b:
            self._switch_mode(Mode.ESTOP)
            return

        # Send velocity command
        teleop_cfg = self.config['teleop']
        vx, vy, omega = self.gamepad.get_velocity_command(
            linear_scale=teleop_cfg['linear_scale'],
            angular_scale=teleop_cfg['angular_scale'],
        )
        self.base_bridge.send(encode_velocity(vx, vy, omega))

    def _auto_update(self):
        """Handle autonomous mode."""
        state = self.mission.update()

        # Check gamepad for ESTOP even in auto mode
        self.gamepad.update()
        if self.gamepad.btn_b:
            self._switch_mode(Mode.ESTOP)

    def _switch_mode(self, new_mode: Mode):
        """Switch operating mode."""
        logger.info(f"Mode switch: {self.mode.name} -> {new_mode.name}")
        self.mode = new_mode

        frame = encode_mode_switch(new_mode)
        self.base_bridge.send(frame)
        self.arm_bridge.send(frame)

        if new_mode == Mode.AUTO:
            self.mission.start()
        elif new_mode == Mode.ESTOP:
            self.mission.stop()
            # Stop base
            self.base_bridge.send(encode_velocity(0.0, 0.0, 0.0))

    def _print_telemetry(self):
        """Print sensor data to terminal (competition requirement)."""
        odom = self.base_bridge.last_odometry
        sensor = self.base_bridge.last_sensor_data

        if odom:
            print(f"\rOdom: x={odom['x']:.3f} y={odom['y']:.3f} "
                  f"θ={odom['theta']:.2f}rad", end="")

        if sensor:
            print(f" | IMU yaw={sensor['imu_yaw']:.2f}° "
                  f"Enc=[{sensor['encoders'][0]},{sensor['encoders'][1]},"
                  f"{sensor['encoders'][2]},{sensor['encoders'][3]}]", end="")

        # Show detected color
        if self.mission.current_cube_color:
            print(f" | Cube: {self.mission.current_cube_color}", end="")

        print("    ", end="\r")


def main():
    config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'config', 'robot_params.yaml'
    )

    node = MasterNode(config_path)

    # Handle SIGINT gracefully
    def signal_handler(sig, frame):
        node.running = False
    signal.signal(signal.SIGINT, signal_handler)

    node.run()


if __name__ == '__main__':
    main()
