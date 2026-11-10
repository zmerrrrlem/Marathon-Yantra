import struct
import time
import math

# Packet configuration
PACKET_FORMAT = "<11f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
SYNC_BYTES = b'\xFF\xFF'

class SerialHandler:
    def __init__(self, serial, portname):
        self.serial = serial
        self.portname = portname
        self.packet = None
        self.first_time = True
        self.data_status = False

    def wait_for_sync(self, timeout=1):
        start = time.time()
        while time.time() - start < timeout:
            byte = self.serial.read(1)
            if not byte:
                continue
            if byte == b'\xFF':
                second = self.serial.read(1)
                if second == b'\xFF':
                    return True
        return False

    def read_packet(self):
        if not self.wait_for_sync():
            print("Waiting for sync...")
            self.first_time = True
            self.data_status = False
            return None

        length_bytes = self.serial.read(1)
        if len(length_bytes) != 1:
            print("Failed to read length byte")
            self.first_time = True
            self.data_status = False
            return None
        length = length_bytes[0]

        if length != PACKET_SIZE + 1:
            print(f"Invalid length {length}")
            self.first_time = True
            self.data_status = False
            return None

        buffer = self.serial.read(length)
        if len(buffer) != length:
            print("Failed to read complete packet")
            self.first_time = True
            self.data_status = False
            return None

        checksum_calc = (~sum(buffer[:-1]) & 0xFF)
        checksum_recv = buffer[-1]
        if checksum_calc != checksum_recv:
            print(f"Checksum mismatch")
            self.first_time = True
            self.data_status = False
            return None
        
        if self.first_time and self.data_status == True:
            print("\nIMU SUBCON: Data Ready\n")
            self.first_time = False

        data = struct.unpack(PACKET_FORMAT, buffer[:-1])
        self.packet = {
            "orientation": data[0:4],
            "angular_velocity": data[4:7],
            "linear_acceleration": data[7:10],
            "button": data[10]
        }
        self.data_status = True
        return self.packet
    
    def get_quaternion(self):
        return {'x': self.packet["orientation"][0], 'y': self.packet["orientation"][1], 'z': self.packet["orientation"][2], 'w': self.packet["orientation"][3]}

    def get_euler(self):
        quat = self.get_quaternion()
        qx, qy, qz, qw = quat['x'], quat['y'], quat['z'], quat['w']

        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return {'x': roll, 'y': pitch, 'z': yaw}

    # rad/s
    def get_gyro_data(self):
        return {'x': self.packet["angular_velocity"][0], 'y': self.packet["angular_velocity"][1], 'z': self.packet["angular_velocity"][2]}
    
    # m/s^2
    def get_accel_data(self):
        return {'x': self.packet["linear_acceleration"][0], 'y': self.packet["linear_acceleration"][1], 'z': self.packet["linear_acceleration"][2]}
    
    def get_button(self):
        return {'button': self.packet["button"]}