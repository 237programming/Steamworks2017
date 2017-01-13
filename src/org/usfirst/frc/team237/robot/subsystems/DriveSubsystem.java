package org.usfirst.frc.team237.robot.subsystems;

import org.usfirst.frc.team237.robot.OI;
import org.usfirst.frc.team237.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveSubsystem extends Subsystem {

	private Pod frontLeft, frontRight, rearLeft, rearRight;
	
	public DriveSubsystem()
	{
		frontRight = new Pod(RobotMap.DriveMap.frontRight, RobotMap.DriveMap.frontRightSteering, 0);
		frontLeft  = new Pod(RobotMap.DriveMap.frontLeft,  RobotMap.DriveMap.frontLeftSteering,  1);
		rearLeft   = new Pod(RobotMap.DriveMap.rearLeft,   RobotMap.DriveMap.rearLeftSteering,   2);
		rearRight  = new Pod(RobotMap.DriveMap.rearRight,  RobotMap.DriveMap.rearRightSteering,  3);
	}

	public void teleopDrive()
	{
		double x, y, rotate;
		x = OI.strafeJoystick.getRawAxis(0);
		y = OI.strafeJoystick.getRawAxis(0);
		rotate = OI.rotateJoystick.getRawAxis(0);
		
		//calculate angle/speed setpoints using 30x30 in. robot
		double L = 30, W = 30;
		double R = Math.sqrt((L * L) * (W * W));
		double A = x - rotate * (L / R);
		double B = x + rotate * (L / R);
		double C = y - rotate * (W / R);
		double D = y + rotate * (W / R);
		
		//find wheel speeds
		double frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
		double frontLeftWheelSpeed  = Math.sqrt((B * B) + (D * D));
		double rearLeftWheelSpeed   = Math.sqrt((A * A) + (D * D));
		double rearRightWheelSpeed  = Math.sqrt((A * A) + (C * C));

		//normalize wheel speeds
		double max = frontRightWheelSpeed;
		if(frontLeftWheelSpeed > max)
		{
			max = frontLeftWheelSpeed;
		}
		if(rearLeftWheelSpeed > max)
		{
			max = rearLeftWheelSpeed;
		}
		if(rearRightWheelSpeed > max)
		{
			max = rearRightWheelSpeed;
		}
		if(max > 1)
		{
			//Max is more than 1.0, so normalize.
			frontRightWheelSpeed /= max;
			frontLeftWheelSpeed  /= max;
			rearLeftWheelSpeed   /= max;
			rearRightWheelSpeed  /= max;
		}
		
		//find steering angles
		double frontRightSteeringAngle = Math.atan2(B, C) * 180/Math.PI;
		double frontLeftSteeringAngle  = Math.atan2(B, D) * 180/Math.PI;
		double rearLeftSteeringAngle   = Math.atan2(A, D) * 180/Math.PI;
		double rearRightSteeringAngle  = Math.atan2(A, C) * 180/Math.PI;

		frontRight.setSteeringAngle (frontRightSteeringAngle);
		frontRight.setWheelSpeed    (frontRightWheelSpeed);
		frontLeft .setSteeringAngle (frontLeftSteeringAngle);
		frontLeft .setWheelSpeed    (frontLeftWheelSpeed);
		rearLeft  .setSteeringAngle (rearLeftSteeringAngle);
		rearLeft  .setWheelSpeed    (rearLeftWheelSpeed);
		rearRight .setSteeringAngle (rearRightSteeringAngle);
		rearRight .setWheelSpeed    (rearRightWheelSpeed);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

