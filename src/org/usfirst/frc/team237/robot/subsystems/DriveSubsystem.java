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
public class DriveSubsystem extends Subsystem implements PIDOutput  {
	
	private double x, y, rotate;
	private Pod pod0, pod1, pod2, pod3;
	private double targetX = 0;
	private double targetY = 0;
	private AHRS gyro; 
	private double angularTarget = 0;
	private double correctionAngle = 0; 
	private double currentX=0, currentY=0; 
	private boolean fieldOriented = false; 
	private boolean inDeadBand = false;
	private double joystickDeadband = 0.05;
	private PIDController angularPID; 
	public DriveSubsystem()
	{
		pod0 = new Pod(RobotMap.DriveMap.pod0, RobotMap.DriveMap.pod0Steering, 0, RobotMap.DriveMap.pod0Offset); //FrontRight 2_____1
		pod1 = new Pod(RobotMap.DriveMap.pod1, RobotMap.DriveMap.pod1Steering, 1, RobotMap.DriveMap.pod1Offset); //FrontLeft  |  ^  |
		pod2 = new Pod(RobotMap.DriveMap.pod2, RobotMap.DriveMap.pod2Steering, 2, RobotMap.DriveMap.pod2Offset); //RearLeft   |  |  |
		pod3 = new Pod(RobotMap.DriveMap.pod3, RobotMap.DriveMap.pod3Steering, 3, RobotMap.DriveMap.pod3Offset); //RearRight  3_____0
	
		// Instantiate gyro for field oriented drive
		gyro = new AHRS(SerialPort.Port.kMXP);
		gyro.reset();
		// instantiate PID for field oriented drive (F.O.D
		angularPID = new PIDController(0.1, 0.001, 0.0, gyro, this); 
		angularPID.disable();
		angularPID.setInputRange(-180, 180);
		angularPID.setOutputRange(-180, 180);
		angularPID.setPercentTolerance(20);
		angularPID.setContinuous();
	}
	/* ---Enable F.O.D.--- */ 
	public void enableFOD()
	{
		angularTarget = gyro.pidGet();
		fieldOriented = true;
		angularPID.setPID(0.1, 0.001, 0.0);
		enableAngularControl();
	}
	/* ---Disable F.O.D.--- */
	public void disableFOD()
	{
		fieldOriented = false; 
		angularTarget = 0;
		angularPID.disable();
	}
	/* ---Check F.O.D.--- */
	public boolean fieldOriented()
	{
		return fieldOriented; 
	}
	/* ---return Drive Train angular target for F.O.D.--- */
	public double angularTarget()
	{
		return angularTarget; 
	}
	/* ---Set F.O.D. target angle--- */
	public void setAngularTarget(double target)
	{
		angularTarget = target; 
	}
	/* ---Enable Angular Correction in Drive train--- */
	public void enableAngularControl()
	{
		angularPID.setSetpoint(angularTarget);
		angularPID.enable();
	}	
	public void enableRotateTo()
	{
		angularPID.disable();
		angularPID.setPID(0.2, 0.0, 0.0);
		enableAngularControl(); 
	}
	public void disableRotateTo()
	{
		angularPID.disable();
	}
	
	public double getYaw()
	{
		return gyro.pidGet();
	}
	
	/* ---pass in a polar vector and angle the robot will move in that direction and rotation ---*/
	public void autoDrive(double mag, double headingTheta, double baseTheta) {
		double x,y; 
		x = mag*Math.cos(Math.toRadians(headingTheta));
		y = mag*Math.sin(Math.toRadians(headingTheta));
		calcWheelsFromRectCoords(x,y,baseTheta);
	}
	public void PIDDrive()
	{
		calcWheelsFromRectCoords(targetX,targetY,correctionAngle); 
	}
	/* ---Drive the Robot in teleop. two joystick input ---*/
	public void teleopDrive()
	{
		x = -OI.strafeJoystick.getX();
		y = -OI.strafeJoystick.getY();
		rotate = -OI.strafeJoystick.getZ();
		
			
			
		//cubic ramping
		x = Math.pow(x, 3);
		y = Math.pow(y, 3);
		//rotate = Math.pow(rotate, 3);
		
		if (Math.abs(x) < joystickDeadband)
			x = 0;
		if (Math.abs(y) < joystickDeadband)
			y = 0;
		
		if (Math.abs(rotate) < joystickDeadband)
			rotate = 0;

		if (Math.abs(y)< joystickDeadband && Math.abs(x)< joystickDeadband && Math.abs(rotate) < joystickDeadband)
		{
			inDeadBand = true; 
		} 
		else {
			inDeadBand = false;
			rotate = MathStuff.mapStickToAngle(rotate/20);
		}
		
		if (fieldOriented){
//			double gyro_degrees = gyro.getYaw(); 
//			double gyro_radians = gyro_degrees * Math.PI/180; 
//			double temp = y * Math.cos(gyro_radians) + x * Math.sin(gyro_radians);
//			x = -y * Math.sin(gyro_radians) + x * Math.cos(gyro_radians);
//			y = temp;
			
			double theta = - gyro.getYaw();
			double[] vector = new double[2];
			vector[0] = x;
			vector[1] = y;
			vector = MathStuff.rotateVector(vector, theta);
			//angularTarget = gyro.getYaw();
			//calcWheelsFromRectCoords(vector[0],vector[1],correctionAngle);
		} else {
			calcWheelsFromRectCoords(x,y,rotate);
		}
		//rotate = 0; 
		//calculate angle/speed setpoints using 30x30 in. robot
		
	}
	public void calcWheelsFromRectCoords(double x, double y, double rotate)
	{
		//calculate angle/speed setpoints using 30x30 in. robot
		double L = 30, W = 30;
		double R = Math.sqrt((L * L) * (W * W));
		double A = x - rotate * (L / R);
		double B = x + rotate * (L / R);
		double C = y - rotate * (W / R);
		double D = y + rotate * (W / R);
		
		//find wheel speeds		
		double pod2WheelSpeed = Math.sqrt((B * B) + (C * C));  // FRONT RIGHT
		double pod1WheelSpeed = Math.sqrt((B * B) + (D * D));  // FRONT LEFT
		double pod0WheelSpeed = Math.sqrt((A * A) + (D * D));  // REAR LEFT
		double pod3WheelSpeed = Math.sqrt((A * A) + (C * C));  // REAR RIGHT

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
		// controls to keep wheels from turning when stick returns to zero 
		if (!inDeadBand ){
			double pod2SteeringAngle = Math.toDegrees(Math.atan2(B, C)); //FRONT RIGHT
			double pod1SteeringAngle = Math.toDegrees(Math.atan2(B, D)); //FRONT LEFT 
			double pod0SteeringAngle = Math.toDegrees(Math.atan2(A, D)); //REAR LEFT 
			double pod3SteeringAngle = Math.toDegrees(Math.atan2(A, C)); //REAR RIGHT
//			SmartDashboard.putNumber("DriveTrain/Pod 0/Angle", pod0SteeringAngle);
//			SmartDashboard.putNumber("DriveTrain/Pod 1/Angle", pod1SteeringAngle);
//			SmartDashboard.putNumber("DriveTrain/Pod 2/Angle", pod2SteeringAngle);
//			SmartDashboard.putNumber("DriveTrain/Pod 3/Angle", pod3SteeringAngle);
			pod0.setSteeringAngle (pod0SteeringAngle);
			pod1.setSteeringAngle (pod1SteeringAngle);
			pod2.setSteeringAngle (pod2SteeringAngle);
			pod3.setSteeringAngle (pod3SteeringAngle);
		}
		else if (fieldOriented ){
			double pod2SteeringAngle = Math.toDegrees(Math.atan2(B, C)); //FRONT RIGHT
			double pod1SteeringAngle = Math.toDegrees(Math.atan2(B, D)); //FRONT LEFT 
			double pod0SteeringAngle = Math.toDegrees(Math.atan2(A, D)); //REAR LEFT 
			double pod3SteeringAngle = Math.toDegrees(Math.atan2(A, C)); //REAR RIGHT
//			SmartDashboard.putNumber("DriveTrain/Pod 0/Angle", pod0SteeringAngle);
//			SmartDashboard.putNumber("DriveTrain/Pod 1/Angle", pod1SteeringAngle);
//			SmartDashboard.putNumber("DriveTrain/Pod 2/Angle", pod2SteeringAngle);
//			SmartDashboard.putNumber("DriveTrain/Pod 3/Angle", pod3SteeringAngle);
			pod0.setSteeringAngle (pod0SteeringAngle);
			pod1.setSteeringAngle (pod1SteeringAngle);
			pod2.setSteeringAngle (pod2SteeringAngle);
			pod3.setSteeringAngle (pod3SteeringAngle);
		}
//		SmartDashboard.putNumber("DriveTrain/Pod 0/Speed", pod0WheelSpeed);
//		SmartDashboard.putNumber("DriveTrain/Pod 1/Speed", pod1WheelSpeed);
//		SmartDashboard.putNumber("DriveTrain/Pod 2/Speed", pod0WheelSpeed);
//		SmartDashboard.putNumber("DriveTrain/Pod 3/Speed", pod1WheelSpeed);
		pod0.setWheelSpeed    (pod0WheelSpeed);
		pod1.setWheelSpeed    (pod1WheelSpeed);
		pod2.setWheelSpeed    (pod2WheelSpeed);
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
		SmartDashboard.putBoolean("DriveTrain/Field Oriented", fieldOriented);
		SmartDashboard.putNumber("DriveTrain/X Ramp:", -x);
		SmartDashboard.putNumber("DriveTrain/X Actual:", OI.strafeJoystick.getX());
		SmartDashboard.putNumber("DriveTrain/Y Ramp:", y);
		SmartDashboard.putNumber("DriveTrain/Y Actual:", -OI.strafeJoystick.getY());
		SmartDashboard.putNumber("DriveTrain/Rotate Ramp:", -rotate);
		SmartDashboard.putNumber("DriveTrain/Rotate Actual:", OI.rotateJoystick.getZ());
		SmartDashboard.putNumber("DriveTrain/Gyro yaw:", gyro.getYaw());
		SmartDashboard.putNumber("DriveTrain/Angular target:", angularTarget);
		SmartDashboard.putNumber("DriveTrain/Correction Angle:", correctionAngle);
		
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
	@Override
	public void pidWrite(double output) {
		correctionAngle = -output; 
	}

}

