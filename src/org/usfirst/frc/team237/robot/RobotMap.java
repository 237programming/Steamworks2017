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
		public static final int frontLeftOffset = 512;
		public static final int frontRightOffset = 422;
		public static final int rearLeftOffset = 874;
		public static final int rearRightOffset =63;
		
		public static final int frontLeft  = 1;
		public static final int frontRight = 2;
		public static final int rearLeft   = 3;
		public static final int rearRight  = 4;
		
		public static final int frontLeftSteering  = 5;
		public static final int frontRightSteering = 6;
		public static final int rearLeftSteering   = 7;
		public static final int rearRightSteering  = 8;
		
	}
	
	public class ControlMap
	{
		public static final int strafeStick = 0;
		public static final int rotateStick = 1;
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
