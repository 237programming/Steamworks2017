package org.usfirst.frc.team237.robot.subsystems;

import org.usfirst.frc.team237.robot.MathStuff;
import org.usfirst.frc.team237.robot.OI;
import org.usfirst.frc.team237.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveSubsystem extends Subsystem implements PIDOutput   {
	
	private Pod frontLeft, frontRight, rearLeft, rearRight;
	private PIDController angularCtrl;
	private AHRS gyro; 
	private double angularTarget = 0;
	private double currentX, currentY; 
	public DriveSubsystem()
	{
		frontRight = new Pod(RobotMap.DriveMap.frontRight, RobotMap.DriveMap.frontRightSteering, 0, RobotMap.DriveMap.frontRightOffset);
		frontLeft  = new Pod(RobotMap.DriveMap.frontLeft,  RobotMap.DriveMap.frontLeftSteering,  1, RobotMap.DriveMap.frontLeftOffset);
		rearLeft   = new Pod(RobotMap.DriveMap.rearLeft,   RobotMap.DriveMap.rearLeftSteering,   2, RobotMap.DriveMap.rearLeftOffset);
		rearRight  = new Pod(RobotMap.DriveMap.rearRight,  RobotMap.DriveMap.rearRightSteering,  3, RobotMap.DriveMap.rearRightOffset);
	
		// Instantiate gyro for field oriented drive
		//gyro = new AHRS(SerialPort.Port.kMXP);
		// instantiate PID for field oriented drive
		//angularCtrl = new PIDController(0.03,0,0,gyro,this);
		//angularCtrl.disable();
	}
	/* ---pass in a polar vector and angle the robot will move in that direction and rotation ---*/
	public void autoDrive(double mag, double theta){
		double x,y; 
		x = mag*Math.cos(Math.toRadians(theta));
		y = mag*Math.sin(Math.toRadians(theta));
		calcWheelsFromRectCoords(x,y,theta);
	}
	/* ---Drive the Robot in teleop. two joystick input ---*/
	public void teleopDrive()
	{
		double x, y, rotate;
		x = OI.strafeJoystick.getX();
		y = -OI.strafeJoystick.getY();
		rotate = MathStuff.mapStickToAngle(OI.rotateJoystick.getRawAxis(0));
		
		//rotate = 0; 
		//calculate angle/speed setpoints using 30x30 in. robot
		calcWheelsFromRectCoords(x,y,rotate); 
	}
	private void calcWheelsFromRectCoords(double x, double y, double rotate)
	{
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
		frontRightWheelSpeed *= RobotMap.DriveMap.maxSpeed;
		frontLeftWheelSpeed  *= RobotMap.DriveMap.maxSpeed;
		rearLeftWheelSpeed   *= RobotMap.DriveMap.maxSpeed;
		rearRightWheelSpeed  *= RobotMap.DriveMap.maxSpeed;
		
		//find steering angles
		double frontRightSteeringAngle = Math.toDegrees(Math.atan2(B, C));
		double frontLeftSteeringAngle  = Math.toDegrees(Math.atan2(B, D));
		double rearLeftSteeringAngle   = Math.toDegrees(Math.atan2(A, D));
		double rearRightSteeringAngle  = Math.toDegrees(Math.atan2(C, A));
		SmartDashboard.putNumber("DriveTrain/Pod 0/Angle", frontRightSteeringAngle);
		SmartDashboard.putNumber("DriveTrain/Pod 1/Angle", frontLeftSteeringAngle);
		SmartDashboard.putNumber("DriveTrain/Pod 2/Angle", frontRightSteeringAngle);
		SmartDashboard.putNumber("DriveTrain/Pod 3/Angle", frontLeftSteeringAngle);
		SmartDashboard.putNumber("DriveTrain/Pod 0/Speed", frontRightWheelSpeed);
		SmartDashboard.putNumber("DriveTrain/Pod 1/Speed", frontLeftWheelSpeed);
		SmartDashboard.putNumber("DriveTrain/Pod 2/Speed", frontRightWheelSpeed);
		SmartDashboard.putNumber("DriveTrain/Pod 3/Speed", frontLeftWheelSpeed);
		frontRight.setSteeringAngle (frontRightSteeringAngle);
		frontRight.setWheelSpeed    (frontRightWheelSpeed);
		frontLeft .setSteeringAngle (frontLeftSteeringAngle);
		frontLeft .setWheelSpeed    (frontLeftWheelSpeed);
		rearLeft  .setSteeringAngle (rearLeftSteeringAngle);
		rearLeft  .setWheelSpeed    (rearLeftWheelSpeed);
		rearRight .setSteeringAngle (rearRightSteeringAngle);
		rearRight .setWheelSpeed    (rearRightWheelSpeed);
	}
	public void testPodClosedLoop(int pod, double speed, double angle)
	{
		if (pod == 0){
			frontRight.setSteeringAngle(angle);
			frontRight.setWheelSpeed(speed);
		}
		else if (pod == 1){
			frontLeft.setSteeringAngle(angle);
			frontLeft.setWheelSpeed(speed);
		}
		else if (pod == 2){
			rearLeft.setSteeringAngle(angle);
			rearLeft.setWheelSpeed(speed);
		}
		else if (pod == 3){
			rearRight.setSteeringAngle(angle);
			rearRight.setWheelSpeed(speed);
		}
	}
	public void testPodPercentVBus(int pod, double speed, double angle)
	{
		if (pod == 0){
			frontRight.setPercentVBusSteer(angle);
			frontRight.setPercentVBusDrive(speed);
		}
		else if (pod == 1){
			frontLeft.setPercentVBusSteer(angle);
			frontLeft.setPercentVBusDrive(speed);
		}
		else if (pod == 2){
			rearLeft.setPercentVBusSteer(angle);
			rearLeft.setPercentVBusDrive(speed);
		}
		else if (pod == 3){
			rearRight.setPercentVBusSteer(angle);
			rearRight.setPercentVBusDrive(speed);
		}
	}
	public void post(){
		rearRight.post();
		frontRight.post();
		rearLeft.post();
		frontLeft.post();
		
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		calcWheelsFromRectCoords(currentX,currentY,output);
	}
}

