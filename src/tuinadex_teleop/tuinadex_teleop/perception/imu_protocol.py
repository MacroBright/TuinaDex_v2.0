#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Low-level serial communication protocol handler for 6-axis IMU.

Implements frame parsing, checksum verification, unit conversion (REP 103),
and thread-safe state caching without external non-standard dependencies.
"""

from dataclasses import dataclass, field
import math
import struct
import threading
import time
from typing import List, Optional, Tuple
import serial

# Standard gravity acceleration constant (REP 103)
GRAVITY_STANDARD: float = 9.80665
DEG_TO_RAD: float = math.pi / 180.0
RAD_TO_DEG: float = 180.0 / math.pi


@dataclass
class ImuDataSnapshot:
    """Thread-safe snapshot of the latest IMU readings."""

    timestamp: float = 0.0
    # Acceleration in m/s^2 (X, Y, Z)
    accel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Angular velocity in rad/s (X, Y, Z)
    gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Magnetometer in Tesla (X, Y, Z)
    mag: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Quaternion (w, x, y, z), normalized
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    # Euler angles in radians (roll, pitch, yaw)
    euler_rad: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Euler angles in degrees (roll, pitch, yaw)
    euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Barometer & environment: height(m), temp(degC), pressure(hPa), contrast(hPa)
    baro: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    # Raw update counter
    update_count: int = 0


class YbImuProtocol:
    """Serial protocol driver for Yahboom 6-axis/9-axis IMU."""

    HEAD1: int = 0x7E
    HEAD2: int = 0x23

    # Protocol Function Codes
    FUNC_VERSION: int = 0x01
    FUNC_REPORT_IMU_RAW: int = 0x04
    FUNC_REPORT_IMU_QUAT: int = 0x16
    FUNC_REPORT_IMU_EULER: int = 0x26
    FUNC_REPORT_BARO: int = 0x32
    FUNC_REPORT_RATE: int = 0x60
    FUNC_ALGO_TYPE: int = 0x61
    FUNC_CALIB_IMU: int = 0x70
    FUNC_CALIB_MAG: int = 0x71
    FUNC_CALIB_BARO: int = 0x72
    FUNC_CALIB_TEMP: int = 0x73
    FUNC_REQUEST_DATA: int = 0x80
    FUNC_RETURN_STATE: int = 0x81
    FUNC_RESET_FLASH: int = 0xA0

    # Conversion ratios
    ACCEL_RATIO_G: float = 16.0 / 32767.0
    GYRO_RATIO_RAD: float = (2000.0 / 32767.0) * DEG_TO_RAD
    MAG_RATIO_UT: float = 800.0 / 32767.0

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout: float = 0.5,
        debug: bool = False,
    ) -> None:
        self.port: str = port
        self.baudrate: int = baudrate
        self.timeout: float = timeout
        self.debug: bool = debug

        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Internal state variables
        self._snapshot = ImuDataSnapshot()
        self._rx_func: int = 0
        self._rx_state: int = 0
        self._firmware_version: Optional[str] = None

    def open(self) -> bool:
        """Open the serial port and start the background receive worker."""
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=0.5,
            )
            self._ser.reset_input_buffer()
            self._running = True
            self._thread = threading.Thread(
                target=self._receive_loop, name="ImuSerialWorker", daemon=True
            )
            self._thread.start()
            if self.debug:
                print(f"[rebot_imu] Serial opened on {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            if self.debug:
                print(f"[rebot_imu] Failed to open serial {self.port}: {e}")
            self._ser = None
            return False

    def close(self) -> None:
        """Stop background worker and close the serial port."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open and self._running

    def get_snapshot(self) -> ImuDataSnapshot:
        """Return a thread-safe copy of the latest IMU readings."""
        with self._lock:
            return ImuDataSnapshot(
                timestamp=self._snapshot.timestamp,
                accel=self._snapshot.accel,
                gyro=self._snapshot.gyro,
                mag=self._snapshot.mag,
                quat=self._snapshot.quat,
                euler_rad=self._snapshot.euler_rad,
                euler_deg=self._snapshot.euler_deg,
                baro=self._snapshot.baro,
                update_count=self._snapshot.update_count,
            )

    def send_command(self, func: int, payload: List[int]) -> bool:
        """Pack and transmit a command frame."""
        if not self.is_open() or self._ser is None:
            return False
        # Frame structure: HEAD1, HEAD2, LEN, FUNC, PAYLOAD..., CHECKSUM
        # Length includes HEAD1 to CHECKSUM (total bytes = 4 + len(payload) + 1)
        length = 4 + len(payload) + 1
        frame = [self.HEAD1, self.HEAD2, length, func] + payload
        checksum = sum(frame) & 0xFF
        frame.append(checksum)
        try:
            self._ser.write(bytes(frame))
            self._ser.flush()
            return True
        except Exception as e:
            if self.debug:
                print(f"[rebot_imu] send_command error: {e}")
            return False

    def request_firmware_version(self, timeout_sec: float = 1.0) -> Optional[str]:
        """Request and return the microcontroller firmware version string."""
        self._firmware_version = None
        # FUNC_REQUEST_DATA (0x80), arg1=FUNC_VERSION (0x01), arg2=0
        if not self.send_command(self.FUNC_REQUEST_DATA, [self.FUNC_VERSION, 0x00]):
            return None
        start = time.time()
        while time.time() - start < timeout_sec:
            if self._firmware_version:
                return self._firmware_version
            time.sleep(0.01)
        return None

    def set_report_rate(self, rate_hz: int) -> bool:
        """Set autonomous reporting frequency [10..100] Hz."""
        rate_hz = max(10, min(100, int(rate_hz)))
        return self.send_command(self.FUNC_REPORT_RATE, [rate_hz, 0x5F])

    def set_algo_type(self, algo_axes: int = 6) -> bool:
        """Set fusion algorithm type: 6 for 6-axis, 9 for 9-axis."""
        if algo_axes not in (6, 9):
            return False
        return self.send_command(self.FUNC_ALGO_TYPE, [algo_axes, 0x5F])

    def trigger_imu_calibration(self) -> bool:
        """Trigger on-board gyroscope and accelerometer zero-point calibration."""
        self._rx_func = 0
        self._rx_state = 0
        return self.send_command(self.FUNC_CALIB_IMU, [0x01, 0x5F])

    def _receive_loop(self) -> None:
        """Worker thread executing state machine byte parser."""
        rx_flag = 0
        data_len = 0
        data_func = 0
        rx_data: List[int] = []
        rx_count = 0

        while self._running and self._ser:
            try:
                waiting = self._ser.in_waiting
                if waiting <= 0:
                    time.sleep(0.002)
                    continue
                chunk = self._ser.read(waiting)
            except Exception as e:
                if self.debug:
                    print(f"[rebot_imu] Serial read error: {e}")
                time.sleep(0.05)
                continue

            for byte in chunk:
                if rx_flag == 0:
                    if byte == self.HEAD1:
                        rx_flag = 1
                elif rx_flag == 1:
                    if byte == self.HEAD2:
                        rx_flag = 2
                    else:
                        rx_flag = 0
                elif rx_flag == 2:
                    data_len = byte
                    if 5 <= data_len <= 64:
                        rx_flag = 3
                    else:
                        rx_flag = 0
                elif rx_flag == 3:
                    data_func = byte
                    rx_data = []
                    rx_count = 4
                    rx_flag = 4
                elif rx_flag == 4:
                    rx_data.append(byte)
                    rx_count += 1
                    if rx_count >= data_len - 1:
                        rx_flag = 5
                elif rx_flag == 5:
                    rx_flag = 0
                    expected_sum = (
                        self.HEAD1 + self.HEAD2 + data_len + data_func + sum(rx_data)
                    ) & 0xFF
                    if byte == expected_sum:
                        self._parse_payload(data_func, rx_data)
                    elif self.debug:
                        print(
                            f"[rebot_imu] Checksum mismatch: got {byte:#02x}, expected {expected_sum:#02x}"
                        )

    def _parse_payload(self, func: int, data: List[int]) -> None:
        """Parse validated data frame payload into the snapshot cache."""
        raw_bytes = bytes(data)
        now = time.time()

        with self._lock:
            # 1. Raw IMU (accel, gyro, mag)
            if func == self.FUNC_REPORT_IMU_RAW and len(raw_bytes) >= 18:
                # 3x int16 accel (scaled to m/s^2)
                raw_ax, raw_ay, raw_az = struct.unpack("<hhh", raw_bytes[0:6])
                ax = raw_ax * self.ACCEL_RATIO_G * GRAVITY_STANDARD
                ay = raw_ay * self.ACCEL_RATIO_G * GRAVITY_STANDARD
                az = raw_az * self.ACCEL_RATIO_G * GRAVITY_STANDARD

                # 3x int16 gyro (scaled to rad/s)
                raw_gx, raw_gy, raw_gz = struct.unpack("<hhh", raw_bytes[6:12])
                gx = raw_gx * self.GYRO_RATIO_RAD
                gy = raw_gy * self.GYRO_RATIO_RAD
                gz = raw_gz * self.GYRO_RATIO_RAD

                # 3x int16 magnetometer (scaled to Tesla: 1 uT = 1e-6 T)
                raw_mx, raw_my, raw_mz = struct.unpack("<hhh", raw_bytes[12:18])
                mx = raw_mx * self.MAG_RATIO_UT * 1e-6
                my = -raw_my * self.MAG_RATIO_UT * 1e-6  # Invert Y to match body frame
                mz = raw_mz * self.MAG_RATIO_UT * 1e-6

                self._snapshot.accel = (ax, ay, az)
                self._snapshot.gyro = (gx, gy, gz)
                self._snapshot.mag = (mx, my, mz)
                self._snapshot.timestamp = now
                self._snapshot.update_count += 1

            # 2. Quaternion (w, x, y, z)
            elif func == self.FUNC_REPORT_IMU_QUAT and len(raw_bytes) >= 16:
                q0, q1, q2, q3 = struct.unpack("<ffff", raw_bytes[0:16])
                # Normalize quaternion to avoid drift
                norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
                if norm > 1e-6:
                    q0 /= norm
                    q1 /= norm
                    q2 /= norm
                    q3 /= norm
                else:
                    q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
                self._snapshot.quat = (q0, q1, q2, q3)
                self._snapshot.timestamp = now
                self._snapshot.update_count += 1

            # 3. Euler angles (roll, pitch, yaw)
            elif func == self.FUNC_REPORT_IMU_EULER and len(raw_bytes) >= 12:
                roll_rad, pitch_rad, yaw_rad = struct.unpack("<fff", raw_bytes[0:12])
                self._snapshot.euler_rad = (roll_rad, pitch_rad, yaw_rad)
                self._snapshot.euler_deg = (
                    roll_rad * RAD_TO_DEG,
                    pitch_rad * RAD_TO_DEG,
                    yaw_rad * RAD_TO_DEG,
                )
                self._snapshot.timestamp = now
                self._snapshot.update_count += 1

            # 4. Barometer & Environment
            elif func == self.FUNC_REPORT_BARO and len(raw_bytes) >= 16:
                height, temp, press, contrast = struct.unpack("<ffff", raw_bytes[0:16])
                self._snapshot.baro = (
                    round(height, 2),
                    round(temp, 2),
                    round(press, 5),
                    round(contrast, 5),
                )

            # 5. Firmware version
            elif func == self.FUNC_VERSION and len(raw_bytes) >= 3:
                vh, vm, vl = struct.unpack("<BBB", raw_bytes[0:3])
                self._firmware_version = f"V{vh}.{vm}.{vl}"

            # 6. Status reply
            elif func == self.FUNC_RETURN_STATE and len(raw_bytes) >= 2:
                self._rx_func, self._rx_state = struct.unpack("<BB", raw_bytes[0:2])
