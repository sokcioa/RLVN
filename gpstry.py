import sys
import time
import os

from vectornav import Sensor, Registers
from vectornav import VnTypes

class SensorDataReader:

    def __init__(self, port_name: str):
        self.port_name = port_name
        self.sensor = Sensor()
        self.binary_output_register = Registers.BinaryOutput1()

    def connect_sensor(self):
        print(f"Attempting to connect to sensor on {self.port_name}...")
        try:
            self.sensor.autoConnect(self.port_name)
            print(f"Successfully connected to sensor on {self.sensor.connectedPortName()} "
                  f"at baud rate {self.sensor.connectedBaudRate()}.")
        except Exception as e:
            print(f"Error connecting to sensor: {e}")
            sys.exit(1)

    def configure_data_output(self):
        print("Configuring sensor data output...")
        try:
            print("Setting GPS antenna offset to [0.0, 0.0, 0.0]...")
            self.sensor.write_gps_antenna_offset([0.0, 0.0, 0.0])
            print("GPS antenna offset configured successfully.")
        except Exception as e:
            print(f"Error setting GPS antenna offset: {e}")
            self.disconnect_sensor()
            sys.exit(1)
            self.binary_output_register.rateDivisor = 1
            self.binary_output_register.asyncMode.serial1 = 1

        self.binary_output_register.gnss.gnss1TimeUtc = 1
        self.binary_output_register.gnss.gps1Tow = 1
        self.binary_output_register.gnss.gps1Week = 1
        self.binary_output_register.gnss.gnss1NumSats = 1
        self.binary_output_register.gnss.gnss1Fix = 1
        self.binary_output_register.gnss.gnss1PosLla = 1
        self.binary_output_register.gnss.gnss1PosEcef = 1
        self.binary_output_register.gnss.gnss1VelNed = 1
        self.binary_output_register.gnss.gnss1VelEcef = 1
        self.binary_output_register.gnss.gnss1Status = 1

        self.binary_output_register.ins.insStatus = 1
        self.binary_output_register.ins.posLla = 1
        self.binary_output_register.ins.posEcef = 1
        self.binary_output_register.ins.velNed = 1
        self.binary_output_register.ins.velEcef = 1

        try:
            self.sensor.writeRegister(self.binary_output_register)
            print("Data output configured successfully. Waiting for data...")
        except Exception as e:
            print(f"Error configuring data output: {e}")
            self.disconnect_sensor()
            sys.exit(1)

    def wait_for_gps_fix(self, timeout_seconds: int = 60):
        print(f"Waiting for a 3D GPS fix (timeout: {timeout_seconds} seconds)...")
        start_time = time.time()

        while time.time() - start_time < timeout_seconds:
            composite_data = self.sensor.getNextMeasurement(True)
            if not composite_data:
                continue

            if composite_data.matchesMessage(self.binary_output_register):
                gnss_data = composite_data.gnss

                if gnss_data.gnss1Fix is not None and gnss_data.gnss1NumSats is not None:
                    current_fix = gnss_data.gnss1Fix
                    num_sats = gnss_data.gnss1NumSats

                    if current_fix >= 3 and num_sats >= 4:
                        print(f"3D GPS fix acquired! (Fix Type: {current_fix}, Satellites: {num_sats})")
                        return True
                    else:
                        print(f"Current Fix: {current_fix}, Satellites: {num_sats}. Still waiting...", end='\r')
            time.sleep(0.1)

        print("\nTimeout reached. 3D GPS fix not acquired.")
        return False

    def read_and_print_data(self, duration_seconds: int = 10):
        print(f"\nStarting data acquisition for {duration_seconds} seconds...")
        start_time = time.time()

        while time.time() - start_time < duration_seconds:
            composite_data = self.sensor.getNextMeasurement(True)
            if not composite_data:
                continue

            if composite_data.matchesMessage(self.binary_output_register):
                lla_data = composite_data.ins.posLla
                if not lla_data:
                    lla_data = composite_data.gnss.gnss1PosLla

                if lla_data:
                    print(f"GPS LLA: Lat={lla_data.lat:.6f}°, Lon={lla_data.lon:.6f}°, Alt={lla_data.alt:.3f} m")

                vel_ned = composite_data.ins.velNed
                if not vel_ned:
                    vel_ned = composite_data.gnss.gnss1VelNed

                if vel_ned:
                    print(f"Velocity (NED): N={vel_ned[0]:.3f} m/s, E={vel_ned[1]:.3f} m/s, D={vel_ned[2]:.3f} m/s")

                pos_ecef = composite_data.ins.posEcef
                if not pos_ecef:
                    pos_ecef = composite_data.gnss.gnss1PosEcef

                if pos_ecef:
                    print(f"Coordinates (ECEF): X={pos_ecef[0]:.3f} m, Y={pos_ecef[1]:.3f} m, Z={pos_ecef[2]:.3f} m")

                print("-" * 30)

            async_error = self.sensor.getAsynchronousError()
            if async_error is not None:
                print(f"Received async error: {int(async_error.error)} - {async_error.message}")

            time.sleep(0.01)

        print("\nData acquisition finished.")

        self.binary_output_register.rateDivisor = 0
        try:
            self.sensor.writeRegister(self.binary_output_register)
            print("Binary output disabled on sensor.")
        except Exception as e:
            print(f"Error disabling binary output: {e}")

        try:
            print("Restoring sensor to factory settings...")
            self.sensor.restoreFactorySettings()
            print("Sensor restored to factory settings.")
        except Exception as e:
            print(f"Error restoring factory settings: {e}")

        self.disconnect_sensor()

        print("Sensor disconnected and script finished.")

    def disconnect_sensor(self):
        try:
            self.sensor.disconnect()
            print("Sensor disconnected.")
        except Exception as e:
            print(f"Error during disconnection: {e}")

if __name__ == '__main__':
    sensor_port = sys.argv[1] if len(sys.argv) > 1 else "COM30"

    reader = SensorDataReader(sensor_port)

    reader.connect_sensor()

    reader.configure_data_output()

    if reader.wait_for_gps_fix(timeout_seconds=90):
        reader.read_and_print_data(duration_seconds=30)
    else:
        print("Could not acquire a 3D GPS fix. Exiting.")
        reader.disconnect_sensor()

    print("Script execution complete.")
