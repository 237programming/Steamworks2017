package org.usfirst.frc.team237.robot;

import org.usfirst.frc.team237.robot.subsystems.Pod;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public class DriveMap
	{
		//
		public static final int pod0Offset = 1017-512; //FrontRight 2_____1
		public static final int pod1Offset = 640-512;  //FrontLeft  |  ^  |
		public static final int pod2Offset = 1015-512; //RearLeft   |  |  |
		public static final int pod3Offset = 583-512;  //RearRight  3_____0
		
		public static final int pod0 = 2;
		public static final int pod1 = 1;
		public static final int pod2 = 3;
		public static final int pod3 = 4;
		
		public static final int pod0Steering = 6;
		public static final int pod1Steering = 5;
		public static final int pod2Steering = 7;
		public static final int pod3Steering = 8;
		
		public static final int maxSpeed = 8000;
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
