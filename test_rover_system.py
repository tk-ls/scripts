#!/usr/bin/env python3
"""
Comprehensive test suite for MicroMelon Rover Control System
Tests all major components including configuration, controllers, and interfaces.
"""

import unittest
import tempfile
import json
import time
import os
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

# Import our modules
from config import RoverConfig, create_default_config_file
from error_handling import (
    RoverLogger, ConnectionError, SensorError, SafetyError,
    ErrorRecoveryManager, PerformanceMonitor, setup_global_logging
)
from rover_base import (
    SensorData, SensorManager, SafetyManager, 
    SimulatorRoverController, BLERoverController,
    create_rover_controller, connect_physical_rover
)


class TestRoverConfig(unittest.TestCase):
    """Test the configuration system"""
    
    def setUp(self):
        self.config = RoverConfig()
        self.temp_dir = tempfile.mkdtemp()
        self.test_config_path = os.path.join(self.temp_dir, "test_config.json")
    
    def tearDown(self):
        # Clean up temp files
        if os.path.exists(self.test_config_path):
            os.remove(self.test_config_path)
        os.rmdir(self.temp_dir)
    
    def test_default_configuration(self):
        """Test default configuration values"""
        self.assertEqual(self.config.simulator_address, "127.0.0.1")
        self.assertEqual(self.config.simulator_port, 9000)
        self.assertTrue(self.config.enable_collision_avoidance)
        self.assertGreater(self.config.max_motor_speed, 0)
        self.assertIsInstance(self.config.led_colors, dict)
        self.assertIn("RED", self.config.led_colors)
    
    def test_config_validation(self):
        """Test configuration validation"""
        # Valid config should pass
        is_valid, errors = self.config.validate()
        self.assertTrue(is_valid)
        self.assertEqual(len(errors), 0)
        
        # Invalid motor speeds
        self.config.max_motor_speed = -10
        is_valid, errors = self.config.validate()
        self.assertFalse(is_valid)
        self.assertGreater(len(errors), 0)
        
        # Reset to valid
        self.config.max_motor_speed = 50
        
        # Invalid gesture threshold
        self.config.gesture_confidence_threshold = 1.5
        is_valid, errors = self.config.validate()
        self.assertFalse(is_valid)
    
    def test_config_save_load(self):
        """Test saving and loading configuration"""
        # Modify some values
        self.config.max_motor_speed = 75
        self.config.simulator_port = 8000
        self.config.enable_collision_avoidance = False
        
        # Save config
        success = self.config.save_to_file(self.test_config_path)
        self.assertTrue(success)
        self.assertTrue(os.path.exists(self.test_config_path))
        
        # Load config
        loaded_config = RoverConfig.load_from_file(self.test_config_path)
        self.assertEqual(loaded_config.max_motor_speed, 75)
        self.assertEqual(loaded_config.simulator_port, 8000)
        self.assertFalse(loaded_config.enable_collision_avoidance)
    
    def test_config_utility_methods(self):
        """Test configuration utility methods"""
        # Motor limits
        limits = self.config.get_motor_limits()
        self.assertEqual(len(limits), 2)
        self.assertEqual(limits[1], self.config.max_motor_speed)
        
        # Camera resolution
        resolution = self.config.get_camera_resolution()
        self.assertEqual(resolution, (self.config.camera_width, self.config.camera_height))
        
        # Gesture confidence
        self.assertTrue(self.config.is_gesture_confident(0.8))
        self.assertFalse(self.config.is_gesture_confident(0.5))


class TestErrorHandling(unittest.TestCase):
    """Test error handling and logging systems"""
    
    def setUp(self):
        self.config = RoverConfig()
        self.config.log_to_file = False  # Disable file logging for tests
        
    def test_rover_exceptions(self):
        """Test custom rover exceptions"""
        # Connection error
        conn_error = ConnectionError("Test connection error", "BLE")
        self.assertEqual(conn_error.error_code, "CONNECTION_ERROR")
        self.assertEqual(conn_error.connection_type, "BLE")
        self.assertTrue(conn_error.recoverable)
        
        # Sensor error
        sensor_error = SensorError("Test sensor error", "ultrasonic")
        self.assertEqual(sensor_error.error_code, "SENSOR_ERROR")
        self.assertEqual(sensor_error.sensor_type, "ultrasonic")
        
        # Safety error
        safety_error = SafetyError("Test safety error")
        self.assertEqual(safety_error.error_code, "SAFETY_ERROR")
        self.assertFalse(safety_error.recoverable)
    
    def test_error_recovery_manager(self):
        """Test error recovery management"""
        manager = ErrorRecoveryManager(self.config)
        
        # Test error handling
        error = SensorError("Test error", "test_sensor")
        result = manager.handle_error(error)
        
        # Check error was recorded
        self.assertEqual(len(manager.error_history), 1)
        self.assertEqual(manager.error_history[0]['error_code'], "SENSOR_ERROR")
        
        # Test error summary
        summary = manager.get_error_summary()
        self.assertEqual(summary['total_errors'], 1)
        self.assertIn('SENSOR_ERROR', summary['error_types'])
    
    def test_performance_monitor(self):
        """Test performance monitoring"""
        monitor = PerformanceMonitor(self.config)
        
        # Test frame monitoring
        start = monitor.start_frame()
        time.sleep(0.01)  # Simulate work
        duration = monitor.end_frame(start)
        
        self.assertGreater(duration, 0.005)
        self.assertEqual(monitor.frame_count, 1)


class TestSensorData(unittest.TestCase):
    """Test sensor data handling"""
    
    def test_sensor_data_creation(self):
        """Test sensor data object creation"""
        data = SensorData(
            ultrasonic=25.5,
            battery_percentage=85,
            battery_voltage=12.3,
            is_flipped=False
        )
        
        self.assertEqual(data.ultrasonic, 25.5)
        self.assertEqual(data.battery_percentage, 85)
        self.assertFalse(data.is_flipped)
    
    def test_sensor_data_to_dict(self):
        """Test sensor data dictionary conversion"""
        data = SensorData(
            ultrasonic=30.0,
            battery_percentage=75,
            ir_left=100,
            ir_right=90
        )
        
        result_dict = data.to_dict()
        
        self.assertEqual(result_dict['ultrasonic'], '30.0')
        self.assertEqual(result_dict['battery_percentage'], '75%')
        self.assertEqual(result_dict['ir_left'], '100')
        self.assertEqual(result_dict['ir_right'], '90')


class TestSafetyManager(unittest.TestCase):
    """Test safety management system"""
    
    def setUp(self):
        self.config = RoverConfig()
        self.config.obstacle_distance_threshold = 20.0
        self.config.battery_low_threshold = 25
        self.safety_manager = SafetyManager(self.config)
    
    def test_obstacle_detection(self):
        """Test obstacle detection safety check"""
        # Safe distance
        safe_data = SensorData(ultrasonic=30.0, battery_percentage=80)
        is_safe, reason = self.safety_manager.check_safety_conditions(safe_data)
        self.assertTrue(is_safe)
        self.assertEqual(reason, "")
        
        # Unsafe distance
        unsafe_data = SensorData(ultrasonic=15.0, battery_percentage=80)
        is_safe, reason = self.safety_manager.check_safety_conditions(unsafe_data)
        self.assertFalse(is_safe)
        self.assertIn("Obstacle", reason)
    
    def test_battery_safety(self):
        """Test battery level safety check"""
        # Good battery
        good_battery = SensorData(ultrasonic=50.0, battery_percentage=50)
        is_safe, reason = self.safety_manager.check_safety_conditions(good_battery)
        self.assertTrue(is_safe)
        
        # Low battery
        low_battery = SensorData(ultrasonic=50.0, battery_percentage=15)
        is_safe, reason = self.safety_manager.check_safety_conditions(low_battery)
        self.assertFalse(is_safe)
        self.assertIn("Low battery", reason)
    
    def test_flip_detection(self):
        """Test rover flip detection"""
        # Upright rover
        upright = SensorData(ultrasonic=25.0, is_flipped=False)
        is_safe, reason = self.safety_manager.check_safety_conditions(upright)
        self.assertTrue(is_safe)
        
        # Flipped rover
        flipped = SensorData(ultrasonic=25.0, is_flipped=True)
        is_safe, reason = self.safety_manager.check_safety_conditions(flipped)
        self.assertFalse(is_safe)
        self.assertIn("flipped", reason)
    
    def test_safety_limits(self):
        """Test motor speed safety limits"""
        safe_data = SensorData(ultrasonic=30.0, battery_percentage=80)
        
        # Normal operation
        left, right, safety_applied = self.safety_manager.apply_safety_limits(25, 25, safe_data)
        self.assertEqual(left, 25)
        self.assertEqual(right, 25)
        self.assertFalse(safety_applied)
        
        # Unsafe conditions should stop motors
        unsafe_data = SensorData(ultrasonic=10.0)
        left, right, safety_applied = self.safety_manager.apply_safety_limits(25, 25, unsafe_data)
        self.assertEqual(left, 0)
        self.assertEqual(right, 0)
        self.assertTrue(safety_applied)
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        self.assertFalse(self.safety_manager.emergency_stop_active)
        
        # Trigger emergency stop
        self.safety_manager.trigger_emergency_stop("Test emergency")
        self.assertTrue(self.safety_manager.emergency_stop_active)
        
        # Reset emergency stop
        self.safety_manager.reset_emergency_stop()
        self.assertFalse(self.safety_manager.emergency_stop_active)


class TestSensorManager(unittest.TestCase):
    """Test sensor reading management"""
    
    def setUp(self):
        self.config = RoverConfig()
        self.config.sensor_update_interval = 0.1  # Fast for testing
    
    @patch('rover_base.Ultrasonic')
    @patch('rover_base.Battery')
    def test_sensor_readings_with_mocks(self, mock_battery, mock_ultrasonic):
        """Test sensor readings with mocked hardware"""
        # Setup mocks
        mock_ultrasonic.read.return_value = 25.5
        mock_battery.readCurrent.return_value = 1000
        mock_battery.readPercentage.return_value = 85
        mock_battery.readVoltage.return_value = 12.3
        
        sensor_manager = SensorManager(self.config)
        
        # Test ultrasonic reading
        ultrasonic_value = sensor_manager.read_ultrasonic()
        self.assertEqual(ultrasonic_value, 25.5)
        
        # Test battery reading
        current, percentage, voltage = sensor_manager.read_battery()
        self.assertEqual(current, 1000)
        self.assertEqual(percentage, 85)
        self.assertEqual(voltage, 12.3)
    
    @patch('rover_base.Ultrasonic')
    def test_sensor_error_handling(self, mock_ultrasonic):
        """Test sensor error handling"""
        # Mock sensor failure
        mock_ultrasonic.read.side_effect = Exception("Sensor failure")
        
        sensor_manager = SensorManager(self.config)
        
        # Should handle error gracefully
        result = sensor_manager.read_ultrasonic()
        # Should return None or cached value
        self.assertIsNone(result)


class TestRoverControllers(unittest.TestCase):
    """Test rover controller classes"""
    
    def setUp(self):
        self.config = RoverConfig()
        self.config.log_to_file = False
    
    def test_controller_factory(self):
        """Test rover controller factory function"""
        # Test simulator controller creation
        sim_controller = create_rover_controller(self.config, 'simulator')
        self.assertIsInstance(sim_controller, SimulatorRoverController)
        
        # Test BLE controller creation
        ble_controller = create_rover_controller(self.config, 'physical')
        self.assertIsInstance(ble_controller, BLERoverController)
        
        # Test invalid type
        with self.assertRaises(ValueError):
            create_rover_controller(self.config, 'invalid')
    
    @patch('rover_base.RoverController')
    def test_simulator_controller(self, mock_rover_controller):
        """Test simulator rover controller"""
        # Create controller
        controller = SimulatorRoverController(self.config)
        
        # Test connection
        mock_rc_instance = Mock()
        mock_rover_controller.return_value = mock_rc_instance
        controller.rc = mock_rc_instance
        
        # Should succeed to connect
        success = controller.connect()
        self.assertTrue(success)
        self.assertTrue(controller.connected)
        
        # Test cleanup
        controller.cleanup()
        mock_rc_instance.stopRover.assert_called_once()
        mock_rc_instance.end.assert_called_once()

    @patch('rover_base.RoverController')
    def test_connect_physical_helper(self, mock_rover_controller):
        """Ensure physical connection helper uses RoverController.connectBLE"""
        mock_rc_instance = Mock()
        mock_rover_controller.return_value = mock_rc_instance

        controller = BLERoverController(self.config, auto_port=1234)
        controller.rc = mock_rc_instance

        success = connect_physical_rover(controller)

        self.assertTrue(success)
        self.assertTrue(controller.connected)
        mock_rc_instance.connectBLE.assert_called_once_with(1234)
        mock_rc_instance.startRover.assert_called_once()


class TestSystemIntegration(unittest.TestCase):
    """Integration tests for the complete system"""
    
    def setUp(self):
        self.config = RoverConfig()
        self.config.log_to_file = False
    
    @patch('rover_base.RoverController')
    def test_full_system_workflow(self, mock_rover_controller):
        """Test complete system workflow"""
        # Mock the rover controller
        mock_rc_instance = Mock()
        mock_rover_controller.return_value = mock_rc_instance
        
        # Create rover controller
        with create_rover_controller(self.config, 'simulator') as rover:
            # Test basic operations
            self.assertTrue(rover.connected)
            
            # Test motor control
            success = rover.set_motor_speeds(20, 20)
            self.assertTrue(success)
            
            # Test LED control
            color = rover.cycle_led_color()
            self.assertIsNotNone(color)
            
            # Test sensor reading
            sensors = rover.get_sensors()
            self.assertIsInstance(sensors, SensorData)
            
            # Test status
            status = rover.get_status()
            self.assertIn('connected', status)
            self.assertIn('sensors', status)
            self.assertIn('safety', status)


class TestCommandLineInterface(unittest.TestCase):
    """Test command line interface components"""
    
    def test_config_creation(self):
        """Test configuration file creation"""
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = os.path.join(temp_dir, "test_rover_config.json")
            
            # Create config
            config = RoverConfig()
            success = config.save_to_file(config_path)
            self.assertTrue(success)
            self.assertTrue(os.path.exists(config_path))
            
            # Verify content
            with open(config_path, 'r') as f:
                data = json.load(f)
                self.assertIn('max_motor_speed', data)
                self.assertIn('simulator_address', data)


def run_comprehensive_tests():
    """Run all tests and generate report"""
    print("=" * 80)
    print("MicroMelon Rover Control System - Comprehensive Test Suite")
    print("=" * 80)
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    test_classes = [
        TestRoverConfig,
        TestErrorHandling,
        TestSensorData,
        TestSafetyManager,
        TestSensorManager,
        TestRoverControllers,
        TestSystemIntegration,
        TestCommandLineInterface
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, buffer=True)
    result = runner.run(suite)
    
    # Generate summary
    print("\n" + "=" * 80)
    print("TEST SUMMARY")
    print("=" * 80)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped)}")
    
    if result.failures:
        print(f"\nFAILURES ({len(result.failures)}):")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback.split(chr(10))[-2]}")
    
    if result.errors:
        print(f"\nERRORS ({len(result.errors)}):")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback.split(chr(10))[-2]}")
    
    success_rate = (result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100
    print(f"\nSuccess rate: {success_rate:.1f}%")
    
    if result.wasSuccessful():
        print("🎉 All tests passed!")
        return True
    else:
        print("❌ Some tests failed!")
        return False


if __name__ == "__main__":
    # Allow running specific test classes
    import sys
    
    if len(sys.argv) > 1:
        # Run specific test
        unittest.main()
    else:
        # Run comprehensive test suite
        success = run_comprehensive_tests()
        sys.exit(0 if success else 1)
