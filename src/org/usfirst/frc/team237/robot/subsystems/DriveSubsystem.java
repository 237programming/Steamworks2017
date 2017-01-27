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
	
	//private Pod frontLeft, frontRight, rearLeft, rearRight;
	private Pod pod0, pod1, pod2, pod3;
	private PIDController angularCtrl;
	private AHRS gyro; 
	private double angularTarget = 0;
	private double currentX, currentY; 
	public DriveSubsystem()
	{
//		frontRight = new Pod(RobotMap.DriveMap.frontRight, RobotMap.DriveMap.frontRightSteering, 0, RobotMap.DriveMap.frontRightOffset);
//		frontLeft  = new Pod(RobotMap.DriveMap.frontLeft,  RobotMap.DriveMap.frontLeftSteering,  1, RobotMap.DriveMap.frontLeftOffset);
//		rearLeft   = new Pod(RobotMap.DriveMap.rearLeft,   RobotMap.DriveMap.rearLeftSteering,   2, RobotMap.DriveMap.rearLeftOffset);
//		rearRight  = new Pod(RobotMap.DriveMap.rearRight,  RobotMap.DriveMap.rearRightSteering,  3, RobotMap.DriveMap.rearRightOffset);

		pod0 = new Pod(RobotMap.DriveMap.pod0, RobotMap.DriveMap.pod0Steering, 0, RobotMap.DriveMap.pod0Offset); //FrontRight 1_____0
		pod1 = new Pod(RobotMap.DriveMap.pod1, RobotMap.DriveMap.pod1Steering, 1, RobotMap.DriveMap.pod1Offset); //FrontLeft  |  ^  |
		pod2 = new Pod(RobotMap.DriveMap.pod2, RobotMap.DriveMap.pod2Steering, 2, RobotMap.DriveMap.pod2Offset); //RearLeft   |  |  |
		pod3 = new Pod(RobotMap.DriveMap.pod3, RobotMap.DriveMap.pod3Steering, 3, RobotMap.DriveMap.pod3Offset); //RearRight  2_____3
	
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
		double pod0WheelSpeed = Math.sqrt((B * B) + (C * C));
		double pod1WheelSpeed = Math.sqrt((B * B) + (D * D));
		double pod2WheelSpeed = Math.sqrt((A * A) + (D * D));
		double pod3WheelSpeed = Math.sqrt((A * A) + (C * C));

		//normalize wheel speeds
		double max = pod0WheelSpeed;
		if(pod1WheelSpeed > max)
		{
			max = pod1WheelSpeed;
		}
		if(pod2WheelSpeed > max)
		{
			max = pod2WheelSpeed;
		}
		if(pod3WheelSpeed > max)
		{
			max = pod3WheelSpeed;
		}
		if(max > 1)
		{
			//Max is more than 1.0, so normalize.
			pod0WheelSpeed /= max;
			pod1WheelSpeed  /= max;
			pod2WheelSpeed   /= max;
			pod3WheelSpeed  /= max;
		}
		pod0WheelSpeed *= RobotMap.DriveMap.maxSpeed;
		pod1WheelSpeed *= RobotMap.DriveMap.maxSpeed;
		pod2WheelSpeed *= RobotMap.DriveMap.maxSpeed;
		pod3WheelSpeed *= RobotMap.DriveMap.maxSpeed;
		
		//find steering angles
		double pod0SteeringAngle = Math.toDegrees(Math.atan2(B, C));
		double pod1SteeringAngle  = Math.toDegrees(Math.atan2(B, D));
		double pod2SteeringAngle   = Math.toDegrees(Math.atan2(A, D));
		double pod3SteeringAngle  = Math.toDegrees(Math.atan2(A, C));
		SmartDashboard.putNumber("DriveTrain/Pod 0/Angle", pod0SteeringAngle);
		SmartDashboard.putNumber("DriveTrain/Pod 1/Angle", pod1SteeringAngle);
		SmartDashboard.putNumber("DriveTrain/Pod 2/Angle", pod0SteeringAngle);
		SmartDashboard.putNumber("DriveTrain/Pod 3/Angle", pod1SteeringAngle);
		SmartDashboard.putNumber("DriveTrain/Pod 0/Speed", pod0WheelSpeed);
		SmartDashboard.putNumber("DriveTrain/Pod 1/Speed", pod1WheelSpeed);
		SmartDashboard.putNumber("DriveTrain/Pod 2/Speed", pod0WheelSpeed);
		SmartDashboard.putNumber("DriveTrain/Pod 3/Speed", pod1WheelSpeed);
		pod0.setSteeringAngle (pod0SteeringAngle);
		pod0.setWheelSpeed    (pod0WheelSpeed);
		pod1.setSteeringAngle (pod1SteeringAngle);
		pod1.setWheelSpeed    (pod1WheelSpeed);
		pod2.setSteeringAngle (pod2SteeringAngle);
		pod2.setWheelSpeed    (pod2WheelSpeed);
		pod3.setSteeringAngle (pod3SteeringAngle);
		pod3.setWheelSpeed    (pod3WheelSpeed);
	}
	public void testPodClosedLoop(int pod, double speed, double angle)
	{
		if (pod == 0){
			pod0.setSteeringAngle(angle);
			pod0.setWheelSpeed(speed);
		}
		else if (pod == 1){
			pod1.setSteeringAngle(angle);
			pod1.setWheelSpeed(speed);
		}
		else if (pod == 2){
			pod2.setSteeringAngle(angle);
			pod2.setWheelSpeed(speed);
		}
		else if (pod == 3){
			pod3.setSteeringAngle(angle);
			pod3.setWheelSpeed(speed);
		}
	}
	public void testPodPercentVBus(int pod, double speed, double angle)
	{
		if (pod == 0){
			pod0.setPercentVBusSteer(angle);
			pod0.setPercentVBusDrive(speed);
		}
		else if (pod == 1){
			pod1.setPercentVBusSteer(angle);
			pod1.setPercentVBusDrive(speed);
		}
		else if (pod == 2){
			pod2.setPercentVBusSteer(angle);
			pod2.setPercentVBusDrive(speed);
		}
		else if (pod == 3){
			pod3.setPercentVBusSteer(angle);
			pod3.setPercentVBusDrive(speed);
		}
	}
	public void post(){
		pod3.post();
		pod0.post();
		pod2.post();
		pod1.post();
		
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

