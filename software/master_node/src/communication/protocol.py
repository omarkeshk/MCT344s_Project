"""
UART Communication Protocol

Frame format:
| START (0xAA) | MSG_ID (1B) | LENGTH (1B) | PAYLOAD (N bytes) | CRC8 (1B) | END (0x55) |

All multi-byte values are little-endian.
"""

import struct
from enum import IntEnum

START_BYTE = 0xAA
END_BYTE = 0x55


class MsgId(IntEnum):
    CMD_VELOCITY = 0x01    # RPi -> Base: vx(f32), vy(f32), omega(f32)
    ODOMETRY = 0x02        # Base -> RPi: x(f32), y(f32), theta(f32), vx(f32), vy(f32)
    CMD_ARM_JOINTS = 0x03  # RPi -> Arm: j1(f32), j2(f32), j3(f32), j4(f32)
    CMD_GRIPPER = 0x04     # RPi -> Arm: state(u8) 0=open, 1=close
    ARM_STATUS = 0x05      # Arm -> RPi: j1-j5(5xf32), gripper_state(u8)
    MODE_SWITCH = 0x06     # RPi -> Both: mode(u8)
    HEARTBEAT = 0x07       # Bidirectional: timestamp(u32), status(u8)
    SENSOR_DATA = 0x08     # Base -> RPi: imu_yaw(f32), enc1-4(4xi32)
    CMD_ARM_CART = 0x09    # RPi -> Arm: x(f32), y(f32), z(f32)
    ERROR = 0x0A           # Any -> RPi: error_code(u8), details


class Mode(IntEnum):
    TELEOP = 0
    AUTO = 1
    ESTOP = 2


# CRC-8/MAXIM lookup table
_CRC8_TABLE = [0] * 256


def _init_crc8_table():
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
        _CRC8_TABLE[i] = crc


_init_crc8_table()


def crc8(data: bytes) -> int:
    """Compute CRC-8/MAXIM over data bytes."""
    crc = 0x00
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def build_frame(msg_id: int, payload: bytes) -> bytes:
    """Build a complete message frame."""
    length = len(payload)
    if length > 255:
        raise ValueError(f"Payload too long: {length} bytes (max 255)")
    crc_data = bytes([msg_id, length]) + payload
    crc = crc8(crc_data)
    return bytes([START_BYTE, msg_id, length]) + payload + bytes([crc, END_BYTE])


def parse_frame(buffer: bytearray) -> tuple:
    """
    Try to parse a frame from the buffer.

    Returns:
        (msg_id, payload, bytes_consumed) if a valid frame is found,
        (None, None, bytes_consumed) if no valid frame found (bytes_consumed
        indicates how many bytes to discard).
    """
    while len(buffer) >= 5:  # Minimum frame: START + ID + LEN(0) + CRC + END
        # Find START byte
        start_idx = -1
        for i in range(len(buffer)):
            if buffer[i] == START_BYTE:
                start_idx = i
                break

        if start_idx == -1:
            return None, None, len(buffer)

        # Discard bytes before START
        if start_idx > 0:
            return None, None, start_idx

        # Check if we have enough bytes for header
        if len(buffer) < 3:
            return None, None, 0

        msg_id = buffer[1]
        length = buffer[2]
        frame_size = 3 + length + 2  # START + ID + LEN + PAYLOAD + CRC + END

        if len(buffer) < frame_size:
            return None, None, 0  # Wait for more data

        # Verify END byte
        if buffer[frame_size - 1] != END_BYTE:
            return None, None, 1  # Bad frame, skip START byte

        # Extract payload and verify CRC
        payload = bytes(buffer[3:3 + length])
        received_crc = buffer[3 + length]
        crc_data = bytes([msg_id, length]) + payload
        expected_crc = crc8(crc_data)

        if received_crc != expected_crc:
            return None, None, 1  # CRC mismatch, skip START byte

        return msg_id, payload, frame_size

    return None, None, 0


# --- Message encoding helpers ---

def encode_velocity(vx: float, vy: float, omega: float) -> bytes:
    """Encode CMD_VELOCITY message."""
    payload = struct.pack('<fff', vx, vy, omega)
    return build_frame(MsgId.CMD_VELOCITY, payload)


def encode_arm_joints(j1: float, j2: float, j3: float, j4: float) -> bytes:
    """Encode CMD_ARM_JOINTS message."""
    payload = struct.pack('<ffff', j1, j2, j3, j4)
    return build_frame(MsgId.CMD_ARM_JOINTS, payload)


def encode_arm_cartesian(x: float, y: float, z: float) -> bytes:
    """Encode CMD_ARM_CART message."""
    payload = struct.pack('<fff', x, y, z)
    return build_frame(MsgId.CMD_ARM_CART, payload)


def encode_gripper(close: bool) -> bytes:
    """Encode CMD_GRIPPER message."""
    payload = struct.pack('<B', 1 if close else 0)
    return build_frame(MsgId.CMD_GRIPPER, payload)


def encode_mode_switch(mode: Mode) -> bytes:
    """Encode MODE_SWITCH message."""
    payload = struct.pack('<B', int(mode))
    return build_frame(MsgId.MODE_SWITCH, payload)


def encode_heartbeat(timestamp_ms: int, status: int) -> bytes:
    """Encode HEARTBEAT message."""
    payload = struct.pack('<IB', timestamp_ms, status)
    return build_frame(MsgId.HEARTBEAT, payload)


# --- Message decoding helpers ---

def decode_odometry(payload: bytes) -> dict:
    """Decode ODOMETRY payload."""
    x, y, theta, vx, vy = struct.unpack('<fffff', payload)
    return {'x': x, 'y': y, 'theta': theta, 'vx': vx, 'vy': vy}


def decode_arm_status(payload: bytes) -> dict:
    """Decode ARM_STATUS payload."""
    j1, j2, j3, j4, j5 = struct.unpack('<fffff', payload[:20])
    gripper_state = struct.unpack('<B', payload[20:21])[0]
    return {
        'joints': [j1, j2, j3, j4, j5],
        'gripper_open': gripper_state == 0
    }


def decode_heartbeat(payload: bytes) -> dict:
    """Decode HEARTBEAT payload."""
    timestamp, status = struct.unpack('<IB', payload)
    return {'timestamp': timestamp, 'status': status}


def decode_sensor_data(payload: bytes) -> dict:
    """Decode SENSOR_DATA payload."""
    imu_yaw, e1, e2, e3, e4 = struct.unpack('<fiiii', payload)
    return {
        'imu_yaw': imu_yaw,
        'encoders': [e1, e2, e3, e4]
    }
