#!/usr/bin/env python3
"""
kinco_bridge.py
───────────────
Architecture (threaded for 50 Hz odometry):

  command_thread  @ 20 Hz  ──► set_position (steering)  ~8 ms
                            ──► set_speed    (traction)  ~8 ms

  main loop (odom) @ 50 Hz ──► read_position (steering) ~8 ms  ┐ sequential,
                            ──► read_speed    (traction) ~8 ms  ┘ per-port locks
                            ──► publish /odom_kinco

  Per-port threading.Lock() prevents serial collisions.
  Two reads take ~16 ms total → fits inside the 20 ms budget for 50 Hz.
"""

import serial
import threading
import time
import rclpy
from rclpy.node import Node
import math
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

# Global node reference used by helper functions and threads
_node: Node = None

# ─── Serial-port locks (one per physical port) ───────────────────────────────
traction_lock = threading.Lock()
steering_lock = threading.Lock()

# ─── Shared command state ─────────────────────────────────────────────────────
latest_cmd      = None
latest_cmd_lock = threading.Lock()

# ─── Constants ───────────────────────────────────────────────────────────────
MAX_RPM = 2000
MIN_RPM = 0.01
MAX_RPS = 3000
MIN_RPS = 5

ENCODER_RES   = 2 ** 16   # 65536 counts/rev
ENCODER_RES_2 = 2 ** 16
GEAR_RATIO    = 1

TRACTION_GEAR  = 16.4
STEERING_GEAR  = 250
WHEEL_DIAMETER = 0.23   # m
WHEEL_BASE     = 1.08   # m

POSITION_ZERO = 750
POSITION_SIDE = 360 * STEERING_GEAR / 4   # motor degrees for 90 deg actual
MAX_ANGLE     = POSITION_ZERO + POSITION_SIDE
MIN_ANGLE     = POSITION_ZERO - POSITION_SIDE

# Odometry covariance matrices (6x6, row-major, 36 elements)
# Kullanılmayan eksenler (z, roll, pitch) için 1e9 → EKF o kanalı ignore eder.
# pose: [x, y, z, roll, pitch, yaw]
ODOM_POSE_COV = [
    0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1e9,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e9,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e9,  0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.1,
]
# twist: [vx, vy, vz, vroll, vpitch, vyaw]
ODOM_TWIST_COV = [
    0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  1e9,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1e9,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e9,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e9,  0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.05,
]

# Serial read delays — at 38400 baud, 10 bytes ~ 2.6 ms.
# 5 ms gives comfortable margin without killing throughput.
SERIAL_SLEEP  = 0.003   # s — after write, before read

position_control = 1
speed_control    = 3
homing_mode      = 6

# ─── Error / status tables ───────────────────────────────────────────────────
ERROR_CODES = {
    0x7380: "ABZ signals error / no communication with encoder",
    0x7331: "ABZ signals error / no communication with encoder",
    0x7381: "UVW signals error / invalid multiturn data (reset procedure required)",
    0x7320: "UVW signals error / invalid multiturn data (reset procedure required)",
    0x7305: "Counting error / encoder communication disturbed",
    0x7330: "Counting error / encoder communication disturbed",
    0x4210: "Heatsink temperature too high",
    0x3210: "DC-Bus over voltage",
    0x3220: "DC-Bus under voltage",
    0x2320: "Power stage or motor short circuit",
    0x2321: "Current ADC full",
    0x7110: "Brake chopper resistor overload",
    0x8611: "Max following error exceeded",
    0x5112: "Logic supply voltage too low",
    0x2350: "Motor or powerstage IIt error",
    0x8A80: "Pulse input frequency too high",
    0x4310: "Motor temperature sensor alarm",
    0x7122: "Wrong UVW signals or motor connection",
    0x6310: "EEPROM checksum fault",
    0x5210: "Current sensor signal offset or ripple too big",
    0x6010: "Software watchdog exception",
    0x6011: "Invalid interrupt exception",
    0x7400: "Wrong MCU type detected",
    0x6320: "No motor data in EEPROM / motor never configured",
    0x6321: "One or more motor wires lost",
    0xFF01: "Dout overload",
    0xFF11: "STO1 input error",
    0xFF12: "STO2 input error",
    0x5443: "DIN pre_enable configured but input inactive",
    0x5442: "Positive limit reached (after homing)",
    0x5441: "Negative limit reached (after homing)",
    0x6012: "Firmware SPI handling internal error",
    0x8100: "CAN connection aborted / service mode timeout",
    0x81FF: "CAN connection aborted / service mode timeout",
    0x8A81: "Motor and encoder direction mismatch",
    0x7382: "ABZ antivalence error (master encoder)",
    0x7306: "Counting error (master encoder)",
    0xFF10: "STO input error",
    0xFF1A: "STO1 abnormal behaviour",
    0xFF1B: "STO2 abnormal behaviour",
}

ERROR_STATE1_BITS = {
    0x0001: "Extended Error (see ErrorState2)",
    0x0002: "Encoder ABZ / encoder not connected",
    0x0004: "Encoder UVW / encoder internal error",
    0x0008: "Encoder counting / CRC error",
    0x0010: "Driver temperature too high",
    0x0020: "DC bus over voltage",
    0x0040: "DC bus under voltage",
    0x0080: "Over current / motor short circuit",
    0x0100: "Brake resistor overload",
    0x0200: "Position following error",
    0x0400: "Low logic voltage",
    0x0800: "Motor or driver IIt",
    0x1000: "Pulse frequency too high",
    0x2000: "Motor temperature alarm",
    0x4000: "Motor commutation / encoder information error",
    0x8000: "EEPROM checksum fault",
}

ERROR_STATE2_BITS = {
    0x0001: "Current sensor signal offset too big",
    0x0002: "Watchdog error",
    0x0004: "Wrong interrupt",
    0x0008: "Wrong MCU type detected",
    0x0010: "Motor not configured",
    0x0020: "Digital output overload",
    0x0040: "STO1 fault",
    0x0080: "STO2 fault",
    0x0100: "External enable inactive",
    0x0200: "Positive limit reached",
    0x0400: "Negative limit reached",
    0x0800: "SPI internal firmware error",
    0x1000: "CAN connection aborted",
    0x2000: "Motor/encoder direction mismatch",
    0x4000: "Master encoder ABZ error",
    0x8000: "Master encoder counting error",
}

STATUSWORD_BITS = {
    0:  "Ready to switch on",
    1:  "Switched on",
    2:  "Operation enabled",
    3:  "Fault",
    4:  "Voltage enabled",
    5:  "Quick stop",
    6:  "Switch on disabled",
    7:  "Warning",
    8:  "Manufacturer specific",
    9:  "Remote",
    10: "Target reached",
    11: "Internal limit active",
}


# =============================================================================
# Low-level packet helpers
# =============================================================================

def calc_lrc(data_bytes):
    total = sum(data_bytes) & 0xFF
    return (~total + 1) & 0xFF

def format_value(data_bytes):
    return int.from_bytes(data_bytes, byteorder="little", signed=True)

def decode_abort_code(code):
    abort_dict = {
        0x05040000: "SDO protocol timeout",
        0x06010000: "Unsupported access",
        0x06010001: "Attempt to read write-only object",
        0x06020000: "Object does not exist",
        0x06090011: "Subindex does not exist",
        0x06070010: "Data type mismatch",
        0x06070012: "Data too long",
        0x06070013: "Data too short",
        0x08000000: "General error",
    }
    return abort_dict.get(code, "Unknown abort code")

def download_command_packet(value, param_address, driver_id=0x01):
    index_msb = (param_address >> 24) & 0xFF
    index_lsb = (param_address >> 16) & 0xFF
    subindex  = (param_address >>  8) & 0xFF
    data_size =  param_address        & 0xFF

    if data_size == 0x10:
        func_code, data_len = 0x2B, 2
    elif data_size == 0x08:
        func_code, data_len = 0x2F, 1
    elif data_size == 0x20:
        func_code, data_len = 0x23, 4
    else:
        raise ValueError(f"Unknown data_size: {data_size:#x}")

    data_bytes = [(value >> (8 * i)) & 0xFF for i in range(data_len)]
    while len(data_bytes) < 4:
        data_bytes.append(0x00)

    packet = [driver_id, func_code, index_lsb, index_msb, subindex] + data_bytes[:4]
    packet.append(calc_lrc(packet))
    return packet

def upload_command_packet(param_address, driver_id=0x01):
    index_msb = (param_address >> 24) & 0xFF
    index_lsb = (param_address >> 16) & 0xFF
    subindex  = (param_address >>  8) & 0xFF
    packet = [driver_id, 0x40, index_lsb, index_msb, subindex,
              0x00, 0x00, 0x00, 0x00]
    packet.append(calc_lrc(packet))
    return packet


# =============================================================================
# Response validation
# =============================================================================

def print_download_debug(response, sent_packet):
    print("---- Response Debug ----")
    labels = ["Driver ID", "Func code", "Index LSB", "Index MSB", "Subindex",
              "Data0", "Data1", "Data2", "Data3", "LRC"]
    for i, lbl in enumerate(labels):
        r = response[i] if i < len(response) else None
        if r is None:
            print(f"{lbl:12}: None")
            continue
        if lbl == "Func code":
            expected = 0x60
            marker = "ok" if r == 0x60 else "FAIL"
        elif lbl == "LRC":
            expected = calc_lrc(response[:9])
            marker = "ok" if r == expected else "FAIL"
        else:
            expected = sent_packet[i]
            marker = "ok" if r == expected else "FAIL"
        print(f"{lbl:12}: {hex(r)}  expected: {hex(expected)}  [{marker}]")
    print("------------------------")

def validate_download_response(response, sent_packet, debug=False):
    if len(response) == 0:
        return False, "Response is empty"
    if len(response) < 10:
        return False, "Incomplete response"

    driver_id, func_code, index_lsb, index_msb, subindex = response[:5]

    if driver_id != sent_packet[0]:
        print_download_debug(response, sent_packet)
        return False, f"Driver ID mismatch: {hex(driver_id)}"

    if func_code not in (0x60, 0x80):
        print_download_debug(response, sent_packet)
        return False, f"Func code invalid: {hex(func_code)}"

    for i, (r, e) in enumerate(zip((index_lsb, index_msb, subindex),
                                    (sent_packet[2], sent_packet[3], sent_packet[4])), 1):
        if r != e:
            print_download_debug(response, sent_packet)
            return False, f"Address byte {i} mismatch"

    if response[9] != calc_lrc(response[:9]):
        print_download_debug(response, sent_packet)
        return False, "LRC mismatch"

    if func_code == 0x80:
        print_download_debug(response, sent_packet)
        return False, "Slave reported error (0x80)"

    if debug:
        print_download_debug(response, sent_packet)
    return True, "Response OK"

def print_upload_debug(response, param_address, expected_func, only_errors=False):
    print("---- Read Response Debug ----")
    if len(response) == 0:
        print("Empty response")
        return
    resp = list(response)
    labels = ["Driver ID", "Func code", "Index LSB", "Index MSB",
              "Subindex", "Data0", "Data1", "Data2", "Data3", "LRC"]
    expected = [resp[0], expected_func,
                (param_address >> 16) & 0xFF, (param_address >> 24) & 0xFF,
                (param_address >>  8) & 0xFF,
                resp[5], resp[6], resp[7], resp[8], calc_lrc(resp[:9])]
    for lbl, r, e in zip(labels, resp, expected):
        match = (r == e)
        if (not only_errors) or (not match):
            marker = "ok" if match else "FAIL"
            print(f"{lbl:12}: {r:3} ({hex(r)})  expected: {hex(e)}  [{marker}]")
    print("----------------------------")

def validate_upload_response(response, param_address, driver_id=0x01, debug=False):
    if len(response) == 0:
        return False, "Response empty", None
    if len(response) < 10:
        return False, f"Incomplete response: {len(response)} bytes", None

    resp_driver_id, resp_func = response[0], response[1]
    index_lsb, index_msb, subindex = response[2], response[3], response[4]

    if resp_driver_id != driver_id:
        return False, f"Driver ID mismatch: {hex(resp_driver_id)}", None

    data_size = param_address & 0xFF
    if data_size == 0x08:
        expected_func = 0x4F
    elif data_size == 0x10:
        expected_func = 0x4B
    elif data_size == 0x20:
        expected_func = 0x43
    else:
        return False, f"Unknown data_size {hex(data_size)}", None

    if resp_func == 0x80:
        return False, "Slave returned ERROR (0x80)", None
    if resp_func != expected_func:
        return False, f"Unexpected func: {hex(resp_func)}, expected {hex(expected_func)}", None

    if (index_lsb != (param_address >> 16) & 0xFF or
        index_msb != (param_address >> 24) & 0xFF or
        subindex  != (param_address >>  8) & 0xFF):
        return False, "Parameter address mismatch", None

    if response[9] != calc_lrc(response[:9]):
        return False, "LRC mismatch", None

    if debug:
        print_upload_debug(response, param_address, expected_func)

    value = int.from_bytes(response[5:9], byteorder="little", signed=True)
    return True, "Response OK", value

def interpret_upload_response(response):
    resp = list(response)
    func = resp[1]
    if func == 0x80:
        abort_code = int.from_bytes(resp[5:9], "little")
        print(f"\nSlave Error - abort: {hex(abort_code)} - {decode_abort_code(abort_code)}")
    elif func not in (0x43, 0x4B, 0x4F):
        print(f"\nUnknown response func: {hex(func)}")


# =============================================================================
# Unit conversions
# =============================================================================

def rpm_to_dec(rpm):
    if abs(rpm) < MIN_RPM:
        rpm = 0
    elif abs(rpm) > MAX_RPM:
        rpm = MAX_RPM if rpm > 0 else -MAX_RPM
    return int((rpm * 512 * ENCODER_RES) / 1875)

def dec_to_rpm(dec_value):
    return (dec_value * 1875) / (512 * ENCODER_RES)

def rpss_to_dec(rpss):
    """DEC = rps/s * 65536 * encoder_res / 4_000_000"""
    if abs(rpss) < MIN_RPS:
        rpss = MIN_RPS
    elif abs(rpss) > MAX_RPS:
        rpss = MAX_RPS if rpss > 0 else -MAX_RPS
    return int((rpss * 65536 * ENCODER_RES) / 4_000_000)

def inc_to_deg(inc):
    """Motor encoder INC -> motor shaft degrees."""
    return (360 * inc) / (ENCODER_RES_2 * GEAR_RATIO)

def deg_to_inc(angle_deg):
    """Motor shaft degrees -> INC (clamped to safe range)."""
    if angle_deg > MAX_ANGLE:
        print(f"Angle {angle_deg:.1f} clamped to +{MAX_ANGLE}")
        angle_deg = MAX_ANGLE
    elif angle_deg < MIN_ANGLE:
        print(f"Angle {angle_deg:.1f} clamped to {MIN_ANGLE}")
        angle_deg = MIN_ANGLE
    return int((angle_deg * ENCODER_RES * GEAR_RATIO) / 360)

def actual_to_motor_deg(actual_deg):
    """
    Robot-frame steering angle (degrees) -> motor shaft position (degrees).
    FIX: Scale = STEERING_GEAR (250), not MAX_ANGLE/90 (258.33).
      0 deg actual -> POSITION_ZERO  (750)
     90 deg actual -> 750 + 90x250  = 23250 = MAX_ANGLE
    """
    return POSITION_ZERO + actual_deg * STEERING_GEAR


# =============================================================================
# Motor commands  (callers must hold the appropriate port lock)
# =============================================================================

def _write_read(motor, packet):
    """Send packet, wait SERIAL_SLEEP, return 10-byte response."""
    motor.write(bytes(packet))
    time.sleep(SERIAL_SLEEP)
    return motor.read(10)

def set_speed(motor, rpm):
    dec  = rpm_to_dec(rpm)
    pkt  = download_command_packet(dec, 0x60FF0020)
    resp = _write_read(motor, pkt)
    validate_download_response(resp, pkt)

def read_speed(motor):
    """Returns motor shaft RPM, or None on failure."""
    pkt  = upload_command_packet(0x606C0020)
    resp = _write_read(motor, pkt)
    valid, msg, value = validate_upload_response(resp, 0x606C0020)
    if valid:
        interpret_upload_response(resp)
        return dec_to_rpm(value)
    print(f"read_speed failed: {msg}")
    interpret_upload_response(resp)
    return None

def set_position(motor, position_motor_deg):
    inc  = deg_to_inc(position_motor_deg)
    pkt  = download_command_packet(inc, 0x607A0020)
    resp = _write_read(motor, pkt)
    validate_download_response(resp, pkt)

def read_position(motor):
    """Returns motor shaft degrees, or None on failure."""
    pkt  = upload_command_packet(0x60630020)
    resp = _write_read(motor, pkt)
    valid, msg, value = validate_upload_response(resp, 0x60630020)
    if valid:
        interpret_upload_response(resp)
        return inc_to_deg(value)
    print(f"read_position failed: {msg}")
    interpret_upload_response(resp)
    return None

def set_operation_mode(motor, mode):
    pkt  = download_command_packet(mode, 0x60600008)
    resp = _write_read(motor, pkt)
    print("Sent:    ", [hex(b) for b in pkt])
    print("Received:", [hex(b) for b in resp])
    valid, msg = validate_download_response(resp, pkt)
    print("Result:", valid, "-", msg)

def set_controlword(motor, controlword):
    """
    Controlword bits:
        bit0: Switch_on    bit1: Enable_voltage  bit2: Quick_stop
        bit3: Enable_op    bit4: Set_Point        bit7: Fault_reset
        bit8: Halt
    Common: Enable=0x0F, Disable=0x06, AbsPos=0x103F, Homing=0x0F->0x1F
    """
    pkt  = download_command_packet(controlword, 0x60400010)
    resp = _write_read(motor, pkt)
    valid, msg = validate_download_response(resp, pkt)
    print("Controlword result:", valid, "-", msg)

def set_position_speed(motor, rpm):
    dec  = rpm_to_dec(rpm)
    pkt  = download_command_packet(dec, 0x60810020)
    resp = _write_read(motor, pkt)
    valid, msg = validate_download_response(resp, pkt)
    print("Position speed result:", valid, "-", msg)

def set_position_accel_decel(motor, acceleration, deceleration):
    pkt  = download_command_packet(rpss_to_dec(acceleration), 0x60830020)
    resp = _write_read(motor, pkt)
    valid, msg = validate_download_response(resp, pkt)
    print("Accel result:", valid, "-", msg)

    pkt  = download_command_packet(rpss_to_dec(deceleration), 0x60840020)
    resp = _write_read(motor, pkt)
    valid, msg = validate_download_response(resp, pkt)
    print("Decel result:", valid, "-", msg)

def encoder_data_reset(motor):
    pkt  = download_command_packet(8, 0x26900008)
    resp = _write_read(motor, pkt)
    print("Sent:    ", [hex(b) for b in pkt])
    print("Received:", [hex(b) for b in resp])
    valid, msg = validate_download_response(resp, pkt)
    print("Encoder reset result:", valid, "-", msg)

def read_parameter(motor, param_address):
    pkt  = upload_command_packet(param_address)
    motor.write(bytes(pkt))
    time.sleep(0.05)
    resp = motor.read(10)
    valid, msg, value = validate_upload_response(resp, param_address)
    if not valid:
        print(f"read_parameter {hex(param_address)} failed: {msg}")
        return None
    return value


# =============================================================================
# Status / diagnostics
# =============================================================================

def get_status_word(response):
    if len(response) < 10:
        return None, None
    status_bytes = response[5:9]
    status_int = int.from_bytes(status_bytes, byteorder="little", signed=False)
    status_hex = ''.join(f"{b:02X}" for b in reversed(status_bytes))
    return status_hex, status_int

def check_target_reached(status_int):
    if status_int is None:
        return False
    return (status_int >> 10) & 1

def decode_statusword(value):
    for bit, desc in STATUSWORD_BITS.items():
        if value & (1 << bit):
            print(f"  bit{bit:2d}: {desc}")

def interpret_error_code(error_value):
    if error_value == 0:
        print("No active error")
        return
    print(f"Error: 0x{error_value:04X} - {ERROR_CODES.get(error_value, 'Unknown')}")

def decode_error_state(value, error_dict, name):
    if value == 0:
        print(f"{name}: No errors")
        return
    print(f"\n{name}: 0x{value:04X}")
    for bitmask, description in error_dict.items():
        if value & bitmask:
            print(f"  {bitmask:04X}: {description}")

def drive_health_monitor(motor):
    print("\n=========== DRIVE HEALTH ===========")
    val = read_parameter(motor, 0x603F0010)
    if val is not None:
        interpret_error_code(val)
    val = read_parameter(motor, 0x26000010)
    if val is not None:
        decode_error_state(val, ERROR_STATE1_BITS, "Error State 1")
    val = read_parameter(motor, 0x26010010)
    if val is not None:
        decode_error_state(val, ERROR_STATE2_BITS, "Error State 2")
    val = read_parameter(motor, 0x60410010)
    if val is not None:
        print(f"\nStatusword: 0x{val:04X}")
        decode_statusword(val)
    print("====================================\n")

def check_position_reach(motor, lock, timeout_s=60):
    """Blocking wait until target reached or timeout."""
    start = time.time()
    while True:
        pkt = upload_command_packet(0x60410010)
        with lock:
            motor.write(bytes(pkt))
            time.sleep(SERIAL_SLEEP)
            resp = motor.read(10)
        status_hex, status_int = get_status_word(resp)
        if status_hex is not None:
            reached = check_target_reached(status_int)
            print(f"Status: {status_hex}  Target reached: {bool(reached)}")
            if reached and status_hex == 'FFFFD437':
                print("Target reached.")
                break
        else:
            print("Invalid response")
        if time.time() - start > timeout_s:
            print("Timeout.")
            break


# =============================================================================
# cmd_vel callback
# =============================================================================

def callback(msg):
    global latest_cmd
    with latest_cmd_lock:
        latest_cmd = msg


# =============================================================================
# Ackermann kinematics helper
# =============================================================================

def compute_commands(vx, wz):
    """
    (vx m/s, wz rad/s) -> (steering_angle_rad, motor_rpm, rotate_in_place).
    Handles forward/backward and rotate-in-place cases.
    """
    if abs(vx) > 1e-3:
        steering_angle  = math.atan2(wz * WHEEL_BASE, vx)
        traction_speed  = vx / math.cos(steering_angle)
        rotate_in_place = False

    elif abs(wz) > 1e-3:
        steering_angle  = math.atan2(wz * WHEEL_BASE, 1e-6)
        traction_speed  = max(-0.3, min(0.3, abs(wz) * WHEEL_BASE))
        rotate_in_place = True

    else:
        return 0.0, 0.0, False

    # Clamp to +-90 deg (flip direction for backward motion)
    if steering_angle > math.pi / 2:
        steering_angle -= math.pi
        traction_speed *= -1
    elif steering_angle < -math.pi / 2:
        steering_angle += math.pi
        traction_speed *= -1

    wheel_rpm = (60.0 * traction_speed) / (math.pi * WHEEL_DIAMETER)
    motor_rpm = wheel_rpm * TRACTION_GEAR

    return steering_angle, motor_rpm, rotate_in_place


# =============================================================================
# Command thread  (20 Hz — writes only, never reads)
# =============================================================================

def command_thread_fn(traction_motor, steering_motor):
    """
    Runs at CMD_RATE Hz.
    Computes Ackermann commands from latest_cmd and writes to motors.
    Each serial port protected by its own lock to avoid race with odom reads.

    Timeline per 50ms period (CMD_RATE=20Hz):
      t=0ms   acquire steering_lock, set_position (~8ms), release
      t=8ms   acquire traction_lock,  set_speed   (~8ms), release
      t=16ms  done — odom reads have 34ms window before next command
    """
    CMD_RATE = 20
    period = 1.0 / CMD_RATE

    while rclpy.ok():
        with latest_cmd_lock:
            cmd = latest_cmd

        if cmd is not None:
            steering_angle, motor_rpm, rotate_in_place = compute_commands(cmd.linear.x, cmd.angular.z)

            with steering_lock:
                set_position(steering_motor,
                             actual_to_motor_deg(math.degrees(steering_angle)))

            if rotate_in_place:
                check_position_reach(steering_motor, steering_lock)

            with traction_lock:
                set_speed(traction_motor, motor_rpm)

        time.sleep(period)


# =============================================================================
# main — odometry loop at 50 Hz (reads only)
# =============================================================================

def main():
    global _node

    rclpy.init()
    _node = rclpy.create_node('kinco_bridge')

    _node.declare_parameter('homing', False)
    HOMING = _node.get_parameter('homing').get_parameter_value().bool_value

    _node.create_subscription(Twist, '/cmd_vel', callback, 10)
    odom_pub       = _node.create_publisher(Odometry, '/odom_kinco', 10)
    tf_broadcaster = tf2_ros.TransformBroadcaster(_node)

    # Spin ROS2 callbacks in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    spin_thread.start()

    # Open serial ports
    traction_motor = serial.Serial(
        '/dev/traction_motor', 38400,
        bytesize=8, parity='N', stopbits=1, timeout=0.05)
    steering_motor = serial.Serial(
        '/dev/steering_motor', 38400,
        bytesize=8, parity='N', stopbits=1, timeout=0.05)

    # Flush any stale bytes left from a previous run
    traction_motor.reset_input_buffer()
    traction_motor.reset_output_buffer()
    steering_motor.reset_input_buffer()
    steering_motor.reset_output_buffer()

    try:
        # ── Startup (single-threaded, no lock contention yet) ─────────────────
        drive_health_monitor(traction_motor)
        drive_health_monitor(steering_motor)

        set_operation_mode(traction_motor, speed_control)
        set_speed(traction_motor, 0)
        encoder_data_reset(steering_motor)

        control_word = 0x103F
        if HOMING:
            set_operation_mode(steering_motor, homing_mode)
            set_controlword(steering_motor, control_word)
            check_position_reach(steering_motor, steering_lock)

        set_position_speed(steering_motor, 3000)
        set_position_accel_decel(steering_motor, 50, 50)
        set_position_accel_decel(traction_motor,  5, 50)

        set_position(steering_motor, actual_to_motor_deg(0))
        set_controlword(steering_motor, control_word)
        set_operation_mode(steering_motor, position_control)
        check_position_reach(steering_motor, steering_lock)

        # ── Launch command thread ──────────────────────────────────────────────
        cmd_thread = threading.Thread(
            target=command_thread_fn,
            args=(traction_motor, steering_motor),
            daemon=True,
            name="cmd_thread")
        cmd_thread.start()
        _node.get_logger().info("Command thread started at 20 Hz")

        # ── Odometry loop at 50 Hz ────────────────────────────────────────────
        pos_x, pos_y, pos_theta = 0.0, 0.0, 0.0
        last_time = _node.get_clock().now()
        rate      = _node.create_rate(50)

        _node.get_logger().info("Odometry loop started at 50 Hz")

        while rclpy.ok():
            current_time = _node.get_clock().now()
            dt = (current_time - last_time).nanoseconds * 1e-9
            last_time = current_time

            # ── Read sensors — acquire per-port locks independently ────────────
            with steering_lock:
                motor_deg   = read_position(steering_motor)
            with traction_lock:
                current_rpm = read_speed(traction_motor)

            if motor_deg is None or current_rpm is None:
                _node.get_logger().warning("Serial read failed — skipping odom cycle")
                rate.sleep()
                continue

            # Skip stale dt (e.g. first cycle after long pause)
            if dt > 0.5:
                rate.sleep()
                continue

            # ── Ackermann odometry kinematics (kingpin offset düzeltmeli) ─────────────
            actual_steering_deg = (motor_deg - POSITION_ZERO) / STEERING_GEAR
            steering_angle      = math.radians(actual_steering_deg)

            # Motor shaft RPM -> wheel surface speed (m/s)
            wheel_rpm   = current_rpm / TRACTION_GEAR
            wheel_speed = wheel_rpm * math.pi * WHEEL_DIAMETER / 60.0

            linear_velocity  = wheel_speed * math.cos(steering_angle)
            angular_velocity = math.tan(steering_angle) * linear_velocity / WHEEL_BASE

            # ── Integrate pose ─────────────────────────────────────────────────
            pos_x     += linear_velocity * math.cos(pos_theta) * dt
            pos_y     += linear_velocity * math.sin(pos_theta) * dt
            pos_theta += angular_velocity * dt

            # ── Quaternion from yaw ────────────────────────────────────────────
            qz = math.sin(pos_theta / 2.0)
            qw = math.cos(pos_theta / 2.0)

            # ── TF broadcast ───────────────────────────────────────────────────
            t = TransformStamped()
            t.header.stamp    = current_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id  = "base_footprint"
            t.transform.translation.x = pos_x
            t.transform.translation.y = pos_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            # tf_broadcaster.sendTransform(t)

            # ── Odometry message ───────────────────────────────────────────────
            odom = Odometry()
            odom.header.stamp    = current_time.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id  = "base_footprint"

            odom.pose.pose.position.x    = pos_x
            odom.pose.pose.position.y    = pos_y
            odom.pose.pose.position.z    = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            odom.twist.twist.linear.x  = linear_velocity
            odom.twist.twist.linear.y  = 0.0
            odom.twist.twist.angular.z = angular_velocity

            odom.pose.covariance  = ODOM_POSE_COV
            odom.twist.covariance = ODOM_TWIST_COV

            odom_pub.publish(odom)

            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        # ── Shutdown — always reached, even on Ctrl+C ──────────────────────
        _node.get_logger().info("Shutting down: stopping motors and closing serial ports")
        try:
            with traction_lock:
                set_speed(traction_motor, 0)
        except Exception:
            pass
        try:
            with steering_lock:
                set_position(steering_motor, actual_to_motor_deg(0))
        except Exception:
            pass
        try:
            traction_motor.flush()
            traction_motor.close()
        except Exception:
            pass
        try:
            steering_motor.flush()
            steering_motor.close()
        except Exception:
            pass

        _node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()