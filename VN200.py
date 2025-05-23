from vectornav import Sensor, Registers
import time
import serial.tools.list_ports

class VN200:
    @staticmethod
    def find_vn200_port():
        """
        Find the port where the VN-200 sensor is connected.
        
        Returns:
            str: Port name where VN-200 is connected, or None if not found
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Try to connect to each port and check if it's a VN-200
            try:
                sensor = Sensor()
                sensor.connect(port.device)
                model_register = Registers.Model()
                sensor.readRegister(model_register)
                if model_register.model.startswith("VN-200"):
                    sensor.disconnect()
                    return port.device
                sensor.disconnect()
            except:
                continue
        return None

    def __init__(self, port_name=None):
        """
        Initialize the VN-200 sensor connection.
        
        Args:
            port_name (str, optional): Serial port name. If None, will auto-detect the port.
        """
        if port_name is None:
            port_name = self.find_vn200_port()
            if port_name is None:
                raise ValueError("No VN-200 sensor found on any available ports")
            print(f"Found VN-200 sensor on port: {port_name}")
            
        self.sensor = Sensor()
        self.sensor.autoConnect(port_name)
        print(f"Connected to {port_name} at {self.sensor.connectedBaudRate()} baud")
        
        # Verify sensor model
        model_register = Registers.Model()
        self.sensor.readRegister(model_register)
        print(f"Sensor Model Number: {model_register.model}")
                                    
        if not model_register.model.startswith("VN-200"):
            raise ValueError(f"Expected VN-200 sensor, but got {model_register.model}")
        
        # Configure async data output for GPS data
        self._configure_async_output()

    def _configure_async_output(self):
        """Configure the sensor's async data output for GPS and velocity data."""
        # Configure async data output type
        async_output = Registers.AsyncOutputType()
        async_output.ador = Registers.AsyncOutputType.Ador.GPS
        async_output.serialPort = Registers.AsyncOutputType.SerialPort.Serial1
        self.sensor.writeRegister(async_output)
        print(f"ADOR Configured")
        
        # Configure async data output frequency (2Hz)
        async_freq = Registers.AsyncOutputFreq()
        async_freq.adof = Registers.AsyncOutputFreq.Adof.Rate2Hz
        async_freq.serialPort = Registers.AsyncOutputFreq.SerialPort.Serial1
        self.sensor.writeRegister(async_freq)
        print("ADOF Configured")
        
        # Configure binary output for velocity and GPS data
        binary_output = Registers.BinaryOutput1()
        binary_output.rateDivisor = 400  # 2Hz output rate
        binary_output.asyncMode.serial1 = 1
        binary_output.gps.timeGps = 1
        binary_output.gps.gpsTow = 1
        binary_output.gps.gpsWeek = 1
        binary_output.gps.numSats = 1
        binary_output.gps.fix = 1
        binary_output.gps.positionGpsLla = 1
        binary_output.gps.velocityGpsNed = 1
        binary_output.attitude.yawPitchRoll = 1  # Enable YPR data
        # Add IMU data
        binary_output.common.timeStartup = 1
        binary_output.common.accel = 1
        binary_output.common.angularRate = 1
        binary_output.common.imu = 12
        self.sensor.writeRegister(binary_output)
        print("Binary output configured with IMU data")
        
    def Get_Velocity(self):
        """
        Get the current velocity data from the sensor.
        
        Returns:
            dict: Dictionary containing velocity components in NED frame
                  {'north': float, 'east': float, 'down': float}
        """
        composite_data = self.sensor.getNextMeasurement()
        if composite_data and composite_data.gps.velocityGpsNed:
            velocity = composite_data.gps.velocityGpsNed
            return {
                'north': velocity[0],
                'east': velocity[1],
                'down': velocity[2]
            }
        return None
        
    def Get_Coordinates(self):
        """
        Get the current GPS coordinates from the sensor.
        
        Returns:
            dict: Dictionary containing latitude, longitude, and altitude
                  {'latitude': float, 'longitude': float, 'altitude': float}
        """
        composite_data = self.sensor.getNextMeasurement()
        if composite_data and composite_data.gps.positionGpsLla:
            position = composite_data.gps.positionGpsLla
            return {
                'latitude': position[0],
                'longitude': position[1],
                'altitude': position[2]
            }
        return None
        
    def Get_GPS_Data(self):
        """
        Get comprehensive GPS data from the sensor.
        
        Returns:
            dict: Dictionary containing GPS information including:
                  - Time of week
                  - GPS week
                  - Number of satellites
                  - Fix type
                  - Position (lat, lon, alt)
                  - Velocity (north, east, down)
        """
        composite_data = self.sensor.getNextMeasurement()
        if composite_data and composite_data.gps:
            gps_data = composite_data.gps
            return {
                'time_gps': gps_data.timeGps,
                'gps_tow': gps_data.gpsTow,
                'gps_week': gps_data.gpsWeek,
                'num_satellites': gps_data.numSats,
                'fix_type': gps_data.fix,
                'position': {
                    'latitude': gps_data.positionGpsLla[0],
                    'longitude': gps_data.positionGpsLla[1],
                    'altitude': gps_data.positionGpsLla[2]
                },
                'velocity': {
                    'north': gps_data.velocityGpsNed[0],
                    'east': gps_data.velocityGpsNed[1],
                    'down': gps_data.velocityGpsNed[2]
                }
            }
        return None

    def Get_YPR(self):
        """
        Get the current Yaw, Pitch, and Roll angles from the sensor.
        
        Returns:
            dict: Dictionary containing yaw, pitch, and roll angles in degrees
                  {'yaw': float, 'pitch': float, 'roll': float}
        """
        composite_data = self.sensor.getNextMeasurement()
        if composite_data and composite_data.attitude.yawPitchRoll:
            ypr = composite_data.attitude.yawPitchRoll
            return {
                'yaw': ypr.yaw,
                'pitch': ypr.pitch,
                'roll': ypr.roll
            }
        return None
        
    def Get_Angular_Rate(self):
        """
        Get the current angular rate (gyroscope) data from the sensor.
        
        Returns:
            dict: Dictionary containing angular rates in degrees per second
                  {'x': float, 'y': float, 'z': float}
                  where:
                  - x: rotation rate around X-axis (roll rate)
                  - y: rotation rate around Y-axis (pitch rate)
                  - z: rotation rate around Z-axis (yaw rate)
        """
        composite_data = self.sensor.getNextMeasurement()
        if composite_data and composite_data.imu.angularRate:
            angular_rate = composite_data.imu.angularRate
            return {
                'x': angular_rate[0],  # Roll rate
                'y': angular_rate[1],  # Pitch rate
                'z': angular_rate[2]   # Yaw rate
            }
        return None

    def Get_Acceleration(self):
        """
        Get the current acceleration data from the sensor.
        
        Returns:
            dict: Dictionary containing accelerometer data in m/s²
                  {'x': float, 'y': float, 'z': float}
                  where:
                  - x: acceleration along X-axis
                  - y: acceleration along Y-axis
                  - z: acceleration along Z-axis
        """
        composite_data = self.sensor.getNextMeasurement()
        if composite_data and composite_data.imu.accel:
            accel = composite_data.imu.accel
            return {
                'x': accel[0],
                'y': accel[1],
                'z': accel[2]
            }
        return None

    def Get_IMU_Data(self):
        """
        Get comprehensive IMU data from the sensor.
        
        Returns:
            dict: Dictionary containing:
                  - Angular rates (degrees/s)
                  - Accelerations (m/s²)
                  - Time since startup (ns)
        """
        composite_data = self.sensor.getNextMeasurement()
        if composite_data and composite_data.imu:
            imu_data = composite_data.imu
            return {
                'time_startup': imu_data.timeStartup,
                'angular_rate': {
                    'x': imu_data.angularRate[0],
                    'y': imu_data.angularRate[1],
                    'z': imu_data.angularRate[2]
                },
                'acceleration': {
                    'x': imu_data.accel[0],
                    'y': imu_data.accel[1],
                    'z': imu_data.accel[2]
                }
            }
        return None
        
    def disconnect(self):
        """Disconnect from the sensor."""
        self.sensor.disconnect()
        print("Sensor disconnected")

# Example usage
if __name__ == "__main__":
    # Create VN-200 instance
    vn = VN200(port_name="COM30")  # Change port name as needed
    
    try:
        # Get velocity data
        velocity = vn.Get_Velocity()
        print("Velocity:", velocity)
        
        # Get coordinates
        coords = vn.Get_Coordinates()
        print("Coordinates:", coords)
        
        # Get comprehensive GPS data
        gps_data = vn.Get_GPS_Data()
        print("GPS Data:", gps_data)
        
        # Get Yaw, Pitch, Roll data
        ypr = vn.Get_YPR()
        print("Yaw, Pitch, Roll:", ypr)

        # Get angular rate data
        angular_rate = vn.Get_Angular_Rate()
        print("Angular Rate:", angular_rate)

        # Get acceleration data
        acceleration = vn.Get_Acceleration()
        print("Acceleration:", acceleration)

        # Get comprehensive IMU data
        imu_data = vn.Get_IMU_Data()
        print("IMU Data:", imu_data)
        
    finally:
        # Always disconnect when done
        vn.disconnect() 
