"""
Base rover controller classes for MicroMelon Rover Control System
Provides unified interface for BLE and simulator connections with safety features.
"""

from abc import ABC
import time
import sys
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass
from collections import deque

try:
    from micromelon import *
except ImportError:
    print("Warning: micromelon library not found. Some features may not work.")
    # Create mock classes for development/testing
    class Motors:
        @staticmethod
        def write(left, right): pass
    class LEDs:
        @staticmethod
        def writeAll(color): pass
        @staticmethod
        def off(): pass
    class Ultrasonic:
        @staticmethod
        def read(): return 50.0
    class Battery:
        @staticmethod
        def readCurrent(): return 1000
        @staticmethod
        def readPercentage(): return 85
        @staticmethod
        def readVoltage(): return 12.5
    class IMU:
        @staticmethod
        def readAccel(n=0): return 0.1
        @staticmethod
        def readGyro(n=0): return 0.05
        @staticmethod
        def readGyroAccum(n=0): return 1.2
        @staticmethod
        def isFlipped(): return False
    class IR:
        @staticmethod
        def readLeft(): return 100
        @staticmethod
        def readRight(): return 100
    class Robot:
        @staticmethod
        def setName(name): pass
    class RoverController:
        def connectBLE(self, port): pass
        def connectIP(self, address, port): pass
        def startRover(self): pass
        def stopRover(self): pass
        def end(self): pass

from config import RoverConfig
from error_handling import (
    RoverLogger, ConnectionError, SensorError, SafetyError,
    retry_on_failure, handle_exceptions, get_error_manager
)


@dataclass
class SensorData:
    """Container for all sensor readings"""
    ultrasonic: Optional[float] = None
    battery_current: Optional[int] = None
    battery_percentage: Optional[int] = None
    battery_voltage: Optional[float] = None
    accel_x: Optional[float] = None
    accel_y: Optional[float] = None
    accel_z: Optional[float] = None
    gyro_x: Optional[float] = None
    gyro_y: Optional[float] = None
    gyro_z: Optional[float] = None
    gyro_accum_x: Optional[float] = None
    gyro_accum_y: Optional[float] = None
    gyro_accum_z: Optional[float] = None
    is_flipped: Optional[bool] = None
    ir_left: Optional[int] = None
    ir_right: Optional[int] = None
    timestamp: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for easy serialization"""
        return {
            'ultrasonic': str(self.ultrasonic) if self.ultrasonic is not None else 'N/A',
            'battery_current': f"{self.battery_current}mA" if self.battery_current is not None else 'N/A',
            'battery_percentage': f"{self.battery_percentage}%" if self.battery_percentage is not None else 'N/A',
            'battery_voltage': f"{self.battery_voltage}V" if self.battery_voltage is not None else 'N/A',
            'accel_x': str(self.accel_x) if self.accel_x is not None else 'N/A',
            'accel_y': str(self.accel_y) if self.accel_y is not None else 'N/A',
            'accel_z': str(self.accel_z) if self.accel_z is not None else 'N/A',
            'gyro_x': str(self.gyro_x) if self.gyro_x is not None else 'N/A',
            'gyro_y': str(self.gyro_y) if self.gyro_y is not None else 'N/A',
            'gyro_z': str(self.gyro_z) if self.gyro_z is not None else 'N/A',
            'gyro_accum_x': str(self.gyro_accum_x) if self.gyro_accum_x is not None else 'N/A',
            'gyro_accum_y': str(self.gyro_accum_y) if self.gyro_accum_y is not None else 'N/A',
            'gyro_accum_z': str(self.gyro_accum_z) if self.gyro_accum_z is not None else 'N/A',
            'flipped': 'Yes' if self.is_flipped else 'No' if self.is_flipped is not None else 'N/A',
            'ir_left': str(self.ir_left) if self.ir_left is not None else 'N/A',
            'ir_right': str(self.ir_right) if self.ir_right is not None else 'N/A'
        }


class SensorManager:
    """Manages all sensor readings with error handling and caching"""
    
    def __init__(self, config: RoverConfig):
        self.config = config
        self.logger = RoverLogger(config).get_logger("SensorManager")
        self.last_readings = SensorData()
        self.last_update_time = 0.0
        self.error_manager = get_error_manager(config)
    
    @retry_on_failure(max_attempts=2, delay=0.1)
    @handle_exceptions(default_return=None)
    def read_ultrasonic(self) -> Optional[float]:
        """Read ultrasonic sensor with error handling"""
        try:
            value = Ultrasonic.read()
            if value != 255:  # 255 indicates sensor error
                return float(value)
            return None
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"Ultrasonic read failed: {e}", "ultrasonic")
            )
            return self.last_readings.ultrasonic
    
    @retry_on_failure(max_attempts=2, delay=0.1)
    @handle_exceptions(default_return=(None, None, None))
    def read_battery(self) -> Tuple[Optional[int], Optional[int], Optional[float]]:
        """Read all battery sensors"""
        try:
            current = Battery.readCurrent()
            percentage = Battery.readPercentage()
            voltage = Battery.readVoltage()
            return current, percentage, voltage
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"Battery read failed: {e}", "battery")
            )
            return (self.last_readings.battery_current, 
                   self.last_readings.battery_percentage,
                   self.last_readings.battery_voltage)
    
    @retry_on_failure(max_attempts=2, delay=0.1)
    @handle_exceptions(default_return=(None, None, None))
    def read_imu_accel(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """Read IMU acceleration data"""
        try:
            accel_x = IMU.readAccel(n=0)
            accel_y = IMU.readAccel(n=1)
            accel_z = IMU.readAccel(n=2)
            return accel_x, accel_y, accel_z
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"IMU accel read failed: {e}", "imu_accel")
            )
            return (self.last_readings.accel_x,
                   self.last_readings.accel_y,
                   self.last_readings.accel_z)
    
    @retry_on_failure(max_attempts=2, delay=0.1)
    @handle_exceptions(default_return=(None, None, None))
    def read_imu_gyro(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """Read IMU gyroscope data"""
        try:
            gyro_x = IMU.readGyro(n=0)
            gyro_y = IMU.readGyro(n=1) 
            gyro_z = IMU.readGyro(n=2)
            return gyro_x, gyro_y, gyro_z
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"IMU gyro read failed: {e}", "imu_gyro")
            )
            return (self.last_readings.gyro_x,
                   self.last_readings.gyro_y,
                   self.last_readings.gyro_z)
    
    @retry_on_failure(max_attempts=2, delay=0.1)
    @handle_exceptions(default_return=(None, None, None))
    def read_imu_gyro_accum(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """Read IMU gyroscope accumulation data"""
        try:
            gyro_accum_x = IMU.readGyroAccum(n=0)
            gyro_accum_y = IMU.readGyroAccum(n=1)
            gyro_accum_z = IMU.readGyroAccum(n=2)
            return gyro_accum_x, gyro_accum_y, gyro_accum_z
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"IMU gyro accum read failed: {e}", "imu_gyro_accum")
            )
            return (self.last_readings.gyro_accum_x,
                   self.last_readings.gyro_accum_y,
                   self.last_readings.gyro_accum_z)
    
    @retry_on_failure(max_attempts=2, delay=0.1)
    @handle_exceptions(default_return=None)
    def read_imu_flipped(self) -> Optional[bool]:
        """Check if rover is flipped"""
        try:
            return IMU.isFlipped()
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"IMU flip check failed: {e}", "imu_flip")
            )
            return self.last_readings.is_flipped
    
    @retry_on_failure(max_attempts=2, delay=0.1)
    @handle_exceptions(default_return=(None, None))
    def read_ir_sensors(self) -> Tuple[Optional[int], Optional[int]]:
        """Read infrared sensors"""
        try:
            ir_left = IR.readLeft()
            ir_right = IR.readRight()
            return ir_left, ir_right
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"IR sensors read failed: {e}", "ir")
            )
            return (self.last_readings.ir_left, self.last_readings.ir_right)
    
    def update_all_sensors(self) -> SensorData:
        """Update all sensor readings"""
        current_time = time.time()
        
        # Check if enough time has passed since last update
        if current_time - self.last_update_time < self.config.sensor_update_interval:
            return self.last_readings
        
        # Read all sensors
        ultrasonic = self.read_ultrasonic()
        battery_current, battery_percentage, battery_voltage = self.read_battery()
        accel_x, accel_y, accel_z = self.read_imu_accel()
        gyro_x, gyro_y, gyro_z = self.read_imu_gyro()
        gyro_accum_x, gyro_accum_y, gyro_accum_z = self.read_imu_gyro_accum()
        is_flipped = self.read_imu_flipped()
        ir_left, ir_right = self.read_ir_sensors()
        
        # Update cached readings
        self.last_readings = SensorData(
            ultrasonic=ultrasonic,
            battery_current=battery_current,
            battery_percentage=battery_percentage,
            battery_voltage=battery_voltage,
            accel_x=accel_x,
            accel_y=accel_y,
            accel_z=accel_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            gyro_accum_x=gyro_accum_x,
            gyro_accum_y=gyro_accum_y,
            gyro_accum_z=gyro_accum_z,
            is_flipped=is_flipped,
            ir_left=ir_left,
            ir_right=ir_right,
            timestamp=current_time
        )
        
        self.last_update_time = current_time
        return self.last_readings


class SafetyManager:
    """Manages rover safety systems and collision avoidance"""
    
    def __init__(self, config: RoverConfig):
        self.config = config
        self.logger = RoverLogger(config).get_logger("SafetyManager")
        self.emergency_stop_active = False
        self.safety_violations = deque(maxlen=10)  # Keep last 10 violations
        
    def check_safety_conditions(self, sensor_data: SensorData) -> Tuple[bool, str]:
        """
        Check all safety conditions
        
        Returns:
            (is_safe, reason_if_not_safe)
        """
        if not self.config.enable_collision_avoidance:
            return True, ""
        
        # Check obstacle distance
        if sensor_data.ultrasonic is not None:
            if sensor_data.ultrasonic < self.config.obstacle_distance_threshold:
                violation = f"Obstacle at {sensor_data.ultrasonic:.1f}cm (threshold: {self.config.obstacle_distance_threshold}cm)"
                self.safety_violations.append(violation)
                return False, violation
        
        # Check battery level
        if sensor_data.battery_percentage is not None:
            if sensor_data.battery_percentage < self.config.battery_low_threshold:
                violation = f"Low battery: {sensor_data.battery_percentage}%"
                self.safety_violations.append(violation)
                return False, violation
        
        # Check if flipped
        if sensor_data.is_flipped:
            violation = "Rover is flipped"
            self.safety_violations.append(violation)
            return False, violation
        
        return True, ""
    
    def apply_safety_limits(self, left_motor: int, right_motor: int, 
                           sensor_data: SensorData) -> Tuple[int, int, bool]:
        """
        Apply safety-limited motor commands
        
        Returns:
            (safe_left_motor, safe_right_motor, safety_applied)
        """
        # Check safety conditions
        is_safe, reason = self.check_safety_conditions(sensor_data)
        
        if not is_safe:
            if self.config.emergency_stop_enabled:
                self.logger.warning(f"Safety stop: {reason}")
                return 0, 0, True
        
        # Apply speed limits
        min_speed, max_speed = self.config.get_motor_limits()
        left_motor = max(min_speed, min(max_speed, left_motor))
        right_motor = max(min_speed, min(max_speed, right_motor))
        
        return left_motor, right_motor, False
    
    def trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop"""
        self.emergency_stop_active = True
        self.logger.critical(f"EMERGENCY STOP: {reason}")
        self.safety_violations.append(f"EMERGENCY: {reason}")
    
    def reset_emergency_stop(self):
        """Reset emergency stop (manual intervention required)"""
        self.emergency_stop_active = False
        self.logger.info("Emergency stop reset")
    
    def get_safety_status(self) -> Dict:
        """Get current safety system status"""
        return {
            'emergency_stop_active': self.emergency_stop_active,
            'collision_avoidance_enabled': self.config.enable_collision_avoidance,
            'recent_violations': list(self.safety_violations),
            'obstacle_threshold': self.config.obstacle_distance_threshold,
            'battery_threshold': self.config.battery_low_threshold
        }


class RoverControllerBase(ABC):
    """Abstract base class for rover controllers"""
    
    def __init__(self, config: RoverConfig):
        self.config = config
        self.logger = RoverLogger(config).get_logger(self.__class__.__name__)
        self.rc = RoverController()
        self.connected = False
        self.sensor_manager = SensorManager(config)
        self.safety_manager = SafetyManager(config)
        self.error_manager = get_error_manager(config)
        
        # Color management
        self.current_color_index = 0
        self.color_list = list(config.led_colors.values())
        self.color_names = list(config.led_colors.keys())
        
        self.logger.info(f"Initialized {self.__class__.__name__}")
    
    def connect(self) -> bool:
        """Connect to rover.

        Subclasses for non-BLE targets (e.g., simulator) should override this. BLE connections
        are handled via `connect_physical_rover` to ensure we always use the shared
        `RoverController` instance (`controller.rc`).
        """
        raise NotImplementedError("Connect method not implemented for this controller")
    
    def get_sensors(self) -> SensorData:
        """Get current sensor readings"""
        return self.sensor_manager.update_all_sensors()
    
    def set_motor_speeds(self, left: int, right: int, apply_safety: bool = True) -> bool:
        """
        Set motor speeds with optional safety checks
        
        Returns:
            bool: True if command was applied, False if blocked by safety
        """
        if not self.connected:
            self.logger.warning("Cannot set motor speeds: not connected")
            return False
        
        if apply_safety:
            sensor_data = self.get_sensors()
            left, right, safety_applied = self.safety_manager.apply_safety_limits(
                left, right, sensor_data
            )
            
            if safety_applied:
                self.logger.info("Motor command modified by safety system")
        
        try:
            Motors.write(left, right)
            return True
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"Motor control failed: {e}", "motors")
            )
            return False
    
    def cycle_led_color(self) -> str:
        """Cycle to next LED color"""
        self.current_color_index = (self.current_color_index + 1) % len(self.color_list)
        return self.set_led_color(self.current_color_index)
    
    def set_led_color(self, color_index: int) -> str:
        """Set LED color by index"""
        try:
            if 0 <= color_index < len(self.color_list):
                # Convert RGB tuple to micromelon format if needed
                color = self.color_list[color_index]
                LEDs.writeAll(color)
                self.current_color_index = color_index
                color_name = self.color_names[color_index]
                self.logger.debug(f"LED color set to {color_name}")
                return color_name
            else:
                self.logger.warning(f"Invalid color index: {color_index}")
                return "ERROR"
        except Exception as e:
            self.error_manager.handle_error(
                SensorError(f"LED control failed: {e}", "led")
            )
            return "ERROR"
    
    def turn_off_leds(self):
        """Turn off all LEDs"""
        try:
            LEDs.off()
            self.logger.debug("LEDs turned off")
        except Exception as e:
            self.logger.error(f"Failed to turn off LEDs: {e}")
    
    def cleanup(self):
        """Safe cleanup with proper error handling"""
        self.logger.info("Starting cleanup...")
        
        try:
            # Stop motors
            Motors.write(0, 0)
            self.logger.debug("Motors stopped")
        except Exception as e:
            self.logger.error(f"Failed to stop motors: {e}")
        
        try:
            # Turn off LEDs
            self.turn_off_leds()
        except Exception as e:
            self.logger.error(f"LED cleanup failed: {e}")
        
        try:
            # Stop rover controller
            if self.connected:
                self.rc.stopRover()
                self.rc.end()
                self.connected = False
                self.logger.debug("Rover controller stopped")
        except Exception as e:
            self.logger.error(f"Failed to stop rover controller: {e}")
        
        self.logger.info("Cleanup complete")
    
    def get_status(self) -> Dict:
        """Get comprehensive rover status"""
        sensor_data = self.get_sensors()
        safety_status = self.safety_manager.get_safety_status()
        
        return {
            'connected': self.connected,
            'current_led_color': self.color_names[self.current_color_index],
            'sensors': sensor_data.to_dict(),
            'safety': safety_status,
            'config': {
                'motor_limits': self.config.get_motor_limits(),
                'sensor_update_interval': self.config.sensor_update_interval,
                'collision_avoidance': self.config.enable_collision_avoidance
            }
        }
    
    def __enter__(self):
        """Context manager entry"""
        if self.connect():
            return self
        raise ConnectionError("Failed to connect to rover")
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit with cleanup"""
        self.cleanup()


class BLERoverController(RoverControllerBase):
    """BLE rover controller for physical rovers"""
    
    def __init__(self, config: RoverConfig, auto_port: int = None):
        super().__init__(config)
        self.auto_port = auto_port


def connect_physical_rover(controller: BLERoverController, port: int = None) -> bool:
    """Connect to a physical rover via BLE using the shared RoverController instance.

    Args:
        controller: The BLE rover controller wrapper.
        port: Optional explicit 4-digit BLE port. Falls back to controller.auto_port or a prompt.
    """
    if controller.connected:
        return True

    attempt_count = 0
    max_attempts = controller.config.connection_retry_attempts

    while attempt_count < max_attempts and not controller.connected:
        try:
            port_value = port or controller.auto_port

            if port_value is None:
                port_input = input("Enter BLE port (4 digits): ")
                if not port_input.isdigit() or len(port_input) != 4:
                    print("Invalid port format. Please enter a 4-digit number.")
                    continue
                port_value = int(port_input)
            else:
                if isinstance(port_value, str):
                    if not port_value.isdigit():
                        raise ValueError("BLE port must be numeric")
                    port_value = int(port_value)

            if not (1000 <= int(port_value) <= 9999):
                raise ValueError("BLE port must be a 4-digit number (1000-9999)")

            controller.logger.info(f"Attempting BLE connection on port {port_value}")
            controller.rc.connectBLE(int(port_value))
            controller.connected = True

            Robot.setName("MicroMelon")
            controller.rc.startRover()

            # Initialize with first LED color
            controller.set_led_color(0)

            controller.logger.info(f"Successfully connected to BLE rover on port {port_value}")
            return True

        except TimeoutError:
            attempt_count += 1
            controller.logger.warning(
                f"Connection timeout (attempt {attempt_count}/{max_attempts})"
            )
            if attempt_count < max_attempts:
                time.sleep(2)

        except Exception as e:
            controller.error_manager.handle_error(
                ConnectionError(f"BLE connection failed: {e}", "BLE")
            )
            return False

    controller.logger.error("Failed to establish BLE connection after all attempts")
    return False


class SimulatorRoverController(RoverControllerBase):
    """Simulator rover controller for testing"""
    
    def connect(self) -> bool:
        """Connect to rover simulator"""
        if self.connected:
            return True
        
        try:
            self.logger.info(f"Connecting to simulator at {self.config.simulator_address}:{self.config.simulator_port}")
            
            self.rc.connectIP(
                address=self.config.simulator_address,
                port=self.config.simulator_port
            )
            self.connected = True
            
            Robot.setName("SimRover")
            self.rc.startRover()
            
            # Initialize with first LED color
            self.set_led_color(0)
            
            self.logger.info("Successfully connected to simulator")
            return True
            
        except Exception as e:
            self.error_manager.handle_error(
                ConnectionError(f"Simulator connection failed: {e}", "IP")
            )
            return False


def create_rover_controller(config: RoverConfig, target_type: str, ble_port: int = None) -> RoverControllerBase:
    """
    Factory function to create appropriate rover controller
    
    Args:
        config: Rover configuration
        target_type: 'physical' or 'simulator'
        ble_port: BLE port for physical rover (optional)
        
    Returns:
        Appropriate rover controller instance
    """
    if target_type.lower() == 'physical':
        return BLERoverController(config, auto_port=ble_port)
    elif target_type.lower() == 'simulator':
        return SimulatorRoverController(config)
    else:
        raise ValueError(f"Unknown target type: {target_type}. Use 'physical' or 'simulator'")


if __name__ == "__main__":
    # Test the rover controller system
    print("Testing MicroMelon Rover Controller System")
    print("=" * 50)
    
    # Test with simulator (safer for testing)
    config = RoverConfig()
    
    try:
        with create_rover_controller(config, 'simulator') as rover:
            print("✓ Simulator connection successful")
            
            # Test sensor readings
            sensors = rover.get_sensors()
            print(f"✓ Sensor readings: ultrasonic={sensors.ultrasonic}")
            
            # Test motor control
            success = rover.set_motor_speeds(20, 20)
            print(f"✓ Motor control: {'success' if success else 'failed'}")
            
            # Test LED control
            color = rover.cycle_led_color()
            print(f"✓ LED control: {color}")
            
            # Test safety system
            rover.safety_manager.trigger_emergency_stop("Test emergency")
            left, right, safety_applied = rover.safety_manager.apply_safety_limits(30, 30, sensors)
            print(f"✓ Safety system: motors limited to {left}, {right}")
            
            # Get status
            status = rover.get_status()
            print(f"✓ Status report: {len(status)} fields")
            
            time.sleep(1)  # Let it run briefly
            
    except Exception as e:
        print(f"Test failed: {e}")
    
    print("Rover controller test complete")
