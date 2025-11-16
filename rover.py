#!/usr/bin/env python3
"""
MicroMelon Rover Control System - Unified CLI Interface
Professional command-line interface for controlling rovers with multiple modes.
"""

import argparse
import sys
import signal
import time
from pathlib import Path
from typing import Optional

from config import RoverConfig, create_default_config_file
from error_handling import setup_global_logging, get_error_manager
from rover_base import create_rover_controller, connect_physical_rover


def create_argument_parser():
    """Create and configure the argument parser"""
    parser = argparse.ArgumentParser(
        description="MicroMelon Rover Control System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --mode keyboard --target simulator
  %(prog)s --mode gui --target physical --port 1234
  %(prog)s --mode gesture --target simulator --model custom_model.pkl
  %(prog)s --config-wizard
  %(prog)s --status
  %(prog)s --test-connection --target simulator
        """
    )
    
    # Main operation mode
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        '--mode', 
        choices=['keyboard', 'gui', 'gesture'], 
        help='Control interface mode'
    )
    
    # Special operations
    mode_group.add_argument(
        '--config-wizard',
        action='store_true',
        help='Run interactive configuration wizard'
    )
    
    mode_group.add_argument(
        '--status',
        action='store_true',
        help='Show rover status and exit'
    )
    
    mode_group.add_argument(
        '--test-connection',
        action='store_true',
        help='Test connection to rover and exit'
    )
    
    mode_group.add_argument(
        '--train-gestures',
        action='store_true',
        help='Train gesture recognition model'
    )
    
    # Target specification
    parser.add_argument(
        '--target', 
        choices=['physical', 'simulator'], 
        help='Target rover type (physical BLE rover or simulator)'
    )
    
    # Connection options
    parser.add_argument(
        '--port', 
        type=int,
        metavar='NNNN',
        help='BLE port for physical rover (4-digit number)'
    )
    
    parser.add_argument(
        '--sim-address',
        default=None,
        metavar='IP',
        help='Simulator IP address (default: from config)'
    )
    
    parser.add_argument(
        '--sim-port',
        type=int,
        metavar='PORT',
        help='Simulator port (default: from config)'
    )
    
    # Model and data options
    parser.add_argument(
        '--model', 
        type=Path,
        metavar='PATH',
        help='Gesture model file path (default: from config)'
    )
    
    parser.add_argument(
        '--record-session',
        metavar='NAME',
        help='Record session data with given name'
    )
    
    # Configuration options
    parser.add_argument(
        '--config',
        type=Path, 
        default='rover_config.json',
        metavar='PATH',
        help='Configuration file path (default: rover_config.json)'
    )
    
    parser.add_argument(
        '--create-config',
        action='store_true',
        help='Create default configuration file and exit'
    )
    
    # Logging and debug options
    parser.add_argument(
        '--log-level',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
        help='Logging level (default: from config)'
    )
    
    parser.add_argument(
        '--no-file-logging',
        action='store_true',
        help='Disable file logging'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose output'
    )
    
    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug mode with detailed logging'
    )
    
    # Safety options
    parser.add_argument(
        '--no-safety',
        action='store_true',
        help='Disable safety systems (USE WITH CAUTION)'
    )
    
    parser.add_argument(
        '--emergency-stop-disabled',
        action='store_true',
        help='Disable emergency stop (NOT RECOMMENDED)'
    )
    
    # Performance options
    parser.add_argument(
        '--no-performance-monitoring',
        action='store_true',
        help='Disable performance monitoring'
    )
    
    parser.add_argument(
        '--frame-limit',
        type=int,
        metavar='FPS',
        help='Limit frame rate (for gesture mode)'
    )
    
    return parser


def connect_rover(controller, target_type: str, ble_port: int = None) -> bool:
    """Establish a connection using the shared RoverController instance."""
    target = (target_type or '').lower()
    if target == 'physical':
        return connect_physical_rover(controller, ble_port)
    elif target == 'simulator':
        return controller.connect()
    else:
        raise ValueError(f"Unknown target type: {target_type}")


def validate_arguments(args):
    """Validate command line arguments"""
    errors = []
    
    # Check that mode and target are specified together (except for special operations)
    special_ops = args.config_wizard or args.status or args.create_config or args.train_gestures
    
    if not special_ops:
        if args.mode and not args.target:
            errors.append("--target must be specified when using --mode")
        elif args.target and not args.mode:
            errors.append("--mode must be specified when using --target")
        elif not args.mode and not args.target and not args.test_connection:
            errors.append("Must specify --mode and --target, or use a special operation")
    
    # Validate port format
    if args.port is not None:
        if not (1000 <= args.port <= 9999):
            errors.append("BLE port must be a 4-digit number (1000-9999)")
    
    # Check BLE port for physical target
    if args.target == 'physical' and args.mode and not args.port:
        print("Warning: No BLE port specified for physical rover. Will prompt during connection.")
    
    # Validate model file
    if args.model and not args.model.exists():
        errors.append(f"Gesture model file not found: {args.model}")
    
    # Check config file
    if args.config and not args.config.exists() and not args.create_config:
        print(f"Warning: Configuration file not found: {args.config}")
        print("Will create default configuration.")
    
    return errors


def run_config_wizard(config: RoverConfig) -> bool:
    """Run interactive configuration wizard"""
    print("\n" + "=" * 60)
    print("MicroMelon Rover Configuration Wizard")
    print("=" * 60)
    print("Configure your rover control system settings.")
    print("Press Enter to keep current value, or type new value.\n")
    
    try:
        # Connection settings
        print("Connection Settings:")
        response = input(f"Default BLE port [{config.default_ble_port}]: ").strip()
        if response:
            config.default_ble_port = int(response)
        
        response = input(f"Simulator address [{config.simulator_address}]: ").strip()
        if response:
            config.simulator_address = response
            
        response = input(f"Simulator port [{config.simulator_port}]: ").strip()
        if response:
            config.simulator_port = int(response)
        
        # Safety settings
        print("\nSafety Settings:")
        response = input(f"Enable collision avoidance? [{config.enable_collision_avoidance}]: ").strip()
        if response.lower() in ['true', 'false', 'yes', 'no', 'y', 'n']:
            config.enable_collision_avoidance = response.lower() in ['true', 'yes', 'y']
        
        response = input(f"Obstacle distance threshold (cm) [{config.obstacle_distance_threshold}]: ").strip()
        if response:
            config.obstacle_distance_threshold = float(response)
        
        response = input(f"Battery low threshold (%) [{config.battery_low_threshold}]: ").strip()
        if response:
            config.battery_low_threshold = int(response)
        
        # Performance settings
        print("\nPerformance Settings:")
        response = input(f"Target FPS [{config.fps_target}]: ").strip()
        if response:
            config.fps_target = int(response)
            
        response = input(f"Sensor update interval (s) [{config.sensor_update_interval}]: ").strip()
        if response:
            config.sensor_update_interval = float(response)
        
        # Camera settings (for gesture mode)
        print("\nCamera Settings (for gesture recognition):")
        response = input(f"Camera resolution width [{config.camera_width}]: ").strip()
        if response:
            config.camera_width = int(response)
            
        response = input(f"Camera resolution height [{config.camera_height}]: ").strip()
        if response:
            config.camera_height = int(response)
        
        # Validate and save
        is_valid, validation_errors = config.validate()
        if not is_valid:
            print("\nConfiguration errors found:")
            for error in validation_errors:
                print(f"  - {error}")
            return False
        
        # Save configuration
        if config.save_to_file():
            print(f"\n✓ Configuration saved successfully!")
            return True
        else:
            print(f"\n✗ Failed to save configuration")
            return False
    
    except KeyboardInterrupt:
        print("\n\nConfiguration wizard cancelled.")
        return False
    except Exception as e:
        print(f"\nError during configuration: {e}")
        return False


def show_status(config: RoverConfig, target_type: str):
    """Show rover status"""
    print("\n" + "=" * 60)
    print("MicroMelon Rover Status")
    print("=" * 60)
    
    controller = create_rover_controller(config, target_type)

    try:
        if not connect_rover(controller, target_type):
            print("Failed to connect to rover for status check")
            return

        status = controller.get_status()

        print(f"Connection: {'✓ Connected' if status['connected'] else '✗ Not connected'}")
        print(f"Current LED Color: {status['current_led_color']}")

        print("\nSensor Readings:")
        sensors = status['sensors']
        print(f"  Ultrasonic: {sensors['ultrasonic']}")
        print(f"  Battery: {sensors['battery_percentage']} ({sensors['battery_voltage']})")
        print(f"  IMU Accel: X={sensors['accel_x']}, Y={sensors['accel_y']}, Z={sensors['accel_z']}")
        print(f"  IMU Gyro: X={sensors['gyro_x']}, Y={sensors['gyro_y']}, Z={sensors['gyro_z']}")
        print(f"  IR Sensors: Left={sensors['ir_left']}, Right={sensors['ir_right']}")
        print(f"  Flipped: {sensors['flipped']}")

        print("\nSafety Status:")
        safety = status['safety']
        print(f"  Emergency Stop: {'ACTIVE' if safety['emergency_stop_active'] else 'Inactive'}")
        print(f"  Collision Avoidance: {'Enabled' if safety['collision_avoidance_enabled'] else 'Disabled'}")
        print(f"  Obstacle Threshold: {safety['obstacle_threshold']}cm")
        print(f"  Battery Threshold: {safety['battery_threshold']}%")

        if safety['recent_violations']:
            print(f"  Recent Violations: {len(safety['recent_violations'])}")
            for violation in safety['recent_violations'][-3:]:  # Show last 3
                print(f"    - {violation}")

        print("\nConfiguration:")
        cfg = status['config']
        print(f"  Motor Limits: {cfg['motor_limits']}")
        print(f"  Sensor Update Interval: {cfg['sensor_update_interval']}s")
        print(f"  Collision Avoidance: {cfg['collision_avoidance']}")

    except Exception as e:
        print(f"Failed to get rover status: {e}")
    finally:
        controller.cleanup()


def test_connection(config: RoverConfig, target_type: str, ble_port: int = None):
    """Test connection to rover"""
    print(f"\nTesting connection to {target_type} rover...")
    
    try:
        controller = create_rover_controller(config, target_type, ble_port)
        
        print("Attempting connection...")
        if connect_rover(controller, target_type, ble_port):
            print("✓ Connection successful!")
            
            # Test basic operations
            print("Testing basic operations...")
            
            # Test sensor reading
            sensors = controller.get_sensors()
            print(f"  ✓ Sensors: ultrasonic={sensors.ultrasonic}")
            
            # Test LED control
            color = controller.cycle_led_color()
            print(f"  ✓ LED control: {color}")
            
            # Test motor control (brief)
            success = controller.set_motor_speeds(10, 10)
            time.sleep(0.1)
            controller.set_motor_speeds(0, 0)
            print(f"  ✓ Motor control: {'success' if success else 'failed'}")
            
            print("\nAll tests passed! Rover is ready for operation.")
            
        else:
            print("✗ Connection failed!")
            return False
            
    except Exception as e:
        print(f"✗ Connection test failed: {e}")
        return False
    
    finally:
        try:
            controller.cleanup()
        except:
            pass
    
    return True


def setup_signal_handlers():
    """Setup signal handlers for graceful shutdown"""
    def signal_handler(signum, frame):
        print(f"\nReceived signal {signum}. Shutting down gracefully...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    if hasattr(signal, 'SIGTERM'):
        signal.signal(signal.SIGTERM, signal_handler)


def main():
    """Main entry point"""
    parser = create_argument_parser()
    args = parser.parse_args()
    
    # Validate arguments
    errors = validate_arguments(args)
    if errors:
        print("Error: Invalid arguments:")
        for error in errors:
            print(f"  {error}")
        print(f"\nUse '{parser.prog} --help' for usage information.")
        sys.exit(1)
    
    # Handle create config
    if args.create_config:
        create_default_config_file()
        sys.exit(0)
    
    # Load configuration
    if not args.config.exists() and not args.config_wizard:
        print(f"Configuration file not found: {args.config}")
        print("Creating default configuration...")
        create_default_config_file()
    
    config = RoverConfig.load_from_file(args.config)
    
    # Override config with command line arguments
    if args.log_level:
        config.log_level = args.log_level
    if args.no_file_logging:
        config.log_to_file = False
    if args.verbose:
        config.log_level = 'INFO'
    if args.debug:
        config.log_level = 'DEBUG'
    if args.sim_address:
        config.simulator_address = args.sim_address
    if args.sim_port:
        config.simulator_port = args.sim_port
    if args.no_safety:
        config.enable_collision_avoidance = False
        print("WARNING: Safety systems disabled!")
    if args.emergency_stop_disabled:
        config.emergency_stop_enabled = False
        print("WARNING: Emergency stop disabled!")
    if args.no_performance_monitoring:
        config.enable_performance_monitoring = False
    if args.frame_limit:
        config.fps_target = args.frame_limit
    if args.model:
        config.gesture_model_path = str(args.model)
    
    # Setup logging
    logger_system = setup_global_logging(config)
    logger = logger_system.get_logger("Main")
    
    # Setup signal handlers
    setup_signal_handlers()
    
    logger.info(f"MicroMelon Rover Control System starting...")
    logger.info(f"Arguments: mode={args.mode}, target={args.target}")
    
    try:
        # Handle special operations
        if args.config_wizard:
            success = run_config_wizard(config)
            sys.exit(0 if success else 1)
        
        elif args.status:
            if not args.target:
                print("Error: --target must be specified with --status")
                sys.exit(1)
            show_status(config, args.target)
            sys.exit(0)
        
        elif args.test_connection:
            if not args.target:
                print("Error: --target must be specified with --test-connection")
                sys.exit(1)
            success = test_connection(config, args.target, args.port)
            sys.exit(0 if success else 1)
        
        elif args.train_gestures:
            print("Launching gesture training system...")
            # Import and run training system
            try:
                from train import HandGestureRecognizer
                recognizer = HandGestureRecognizer()
                # Run training interface
                print("Training mode not yet implemented in new architecture.")
                print("Please use 'python train.py' for now.")
            except ImportError:
                print("Gesture training system not available.")
            sys.exit(0)
        
        # Normal operation mode
        elif args.mode and args.target:
            logger.info(f"Starting {args.mode} mode with {args.target} target")
            
            # Import appropriate interface module
            if args.mode == 'keyboard':
                from interfaces.keyboard_interface import KeyboardInterface
                interface_class = KeyboardInterface
            elif args.mode == 'gui':
                from interfaces.gui_interface import GUIInterface
                interface_class = GUIInterface
            elif args.mode == 'gesture':
                from interfaces.gesture_interface import GestureInterface
                interface_class = GestureInterface
            
            controller = create_rover_controller(config, args.target, args.port)

            if not connect_rover(controller, args.target, args.port):
                logger.error("Failed to connect to rover")
                sys.exit(1)

            logger.info("Rover connected successfully")

            try:
                # Create and run interface
                interface = interface_class(controller, config)

                if args.record_session:
                    logger.info(f"Session recording enabled: {args.record_session}")
                    # Enable session recording
                    interface.enable_session_recording(args.record_session)

                logger.info("Starting interface...")
                interface.run()
            finally:
                controller.cleanup()
                
        else:
            print("Error: No operation specified.")
            parser.print_help()
            sys.exit(1)
    
    except KeyboardInterrupt:
        logger.info("Operation cancelled by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()
        sys.exit(1)
    
    logger.info("MicroMelon Rover Control System shutdown complete")


if __name__ == "__main__":
    main()
