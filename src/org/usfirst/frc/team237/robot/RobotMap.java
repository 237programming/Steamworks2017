package org.usfirst.frc.team237.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public class DriveMap
	{
		//Offsets rotation on pods
		public static final int pod0Offset = 700-512;  //FrontRight 1_____0
		public static final int pod1Offset = 945-512;  //FrontLeft  |  ^  |
		public static final int pod2Offset = 548-512;  //RearLeft   |  |  |
		public static final int pod3Offset = 544-512;  //RearRight  2_____3
		
		//CAN Addresses for pod drive motors
		public static final int pod0 = 1;
		public static final int pod1 = 3;
		public static final int pod2 = 4;
		public static final int pod3 = 2;
		
		//CAN Addresses for pod rotation motors
		public static final int pod0Steering = 5;
		public static final int pod1Steering = 7;
		public static final int pod2Steering = 8;
		public static final int pod3Steering = 6;
		
		public static final int maxSpeed = 8000;
	}
	
	public class PIDMap
	{
		public static final double FOD_P = 0.1;
		public static final double FOD_I = 0.001;
		public static final double FOD_D = 0;
		
		public static final double ANG_P = 0.1;
		public static final double ANG_I = 0;
		public static final double ANG_D = 0;
	}
	
	public class ShooterMap
	{
		public static final int shooterTalon = 9;
		public static final int feederTalon  = 10;
		public static final int intakeTalon  = 11;
		public static final int hangerTalon  = 12;
	}
	
	public class ControlMap
	{
		public static final int strafeStick = 0;
		public static final int rotateStick = 1;
		public static final double joystickDeadband = 0.05;
	}
	
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
