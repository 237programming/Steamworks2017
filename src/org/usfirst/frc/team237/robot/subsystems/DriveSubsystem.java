package org.usfirst.frc.team237.robot.subsystems;

import org.usfirst.frc.team237.robot.MathStuff;
import org.usfirst.frc.team237.robot.OI;
import org.usfirst.frc.team237.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends Subsystem implements PIDOutput {
	
	private AHRS          gyro;
	private PodSubsystem  pod0, pod1, pod2, pod3;
	private PIDController angularPID;
	Relay light = new Relay(1);
	
	private double   x, y, rotate;
	private double   targetX = 0, targetY  = 0;
	private double   angularTarget = 0;
	private double   PIDOutput = 0; 
	private double   joystickDeadband = RobotMap.ControlMap.joystickDeadband;
	// flags 
	private boolean  fieldOriented = false;
	private boolean  whileRotating = false;
	private boolean  inDeadBand    = true;
	public  boolean  autoDriving   = false;
	private boolean  fieldRotating = false;
	private boolean	 lowPowerMode = false; 
	
	private DigitalInput digitalIn;
	private AnalogInput analogIn;
	public DriveSubsystem()
	{
		pod0 = new PodSubsystem(RobotMap.DriveMap.pod0, RobotMap.DriveMap.pod0Steering, 0, RobotMap.DriveMap.pod0Offset); //FrontRight 2_____1
		pod1 = new PodSubsystem(RobotMap.DriveMap.pod1, RobotMap.DriveMap.pod1Steering, 1, RobotMap.DriveMap.pod1Offset); //FrontLeft  |  ^  |
		pod2 = new PodSubsystem(RobotMap.DriveMap.pod2, RobotMap.DriveMap.pod2Steering, 2, RobotMap.DriveMap.pod2Offset); //RearLeft   |  |  |
		pod3 = new PodSubsystem(RobotMap.DriveMap.pod3, RobotMap.DriveMap.pod3Steering, 3, RobotMap.DriveMap.pod3Offset); //RearRight  3_____0
	
		//Instantiate gyro for field oriented drive
		gyro = new AHRS(SerialPort.Port.kUSB, AHRS.SerialDataType.kProcessedData, (byte) 200);
		gyro.reset();
		
		//instantiate PID for field oriented drive (F.O.D.)
		angularPID = new PIDController(RobotMap.PIDMap.FOD_P, RobotMap.PIDMap.FOD_I, RobotMap.PIDMap.FOD_D, gyro, this); 
		angularPID.disable();
		angularPID.setInputRange(-180, 180);
		angularPID.setOutputRange(-180, 180);
		angularPID.setPercentTolerance(20);
		angularPID.setContinuous();
		// instantiate the analog and digital inputs 
		digitalIn = new DigitalInput(0);
		analogIn = new AnalogInput(0);
	}
	public void enableLowPower(){
		lowPowerMode = true;
		
	}
	public void disableLowPower(){
		lowPowerMode = false; 
		
	}
	public void lightOn()
	{
		if(light.get() != Relay.Value.kForward) light.set(Relay.Value.kForward);
	}
	
	public void lightOff()
	{
		if(light.get() != Relay.Value.kOff) light.set(Relay.Value.kOff);
	}
	
	public void toggleLight()
	{
		if(light.get() == Relay.Value.kForward) lightOff();
		else if(light.get() == Relay.Value.kOff) lightOn();
		else lightOff();
	}
	
	//Enable F.O.D.
	public void enableFOD()
	{
		angularPID.disable();
		angularTarget = gyro.pidGet();
		fieldOriented = true;
		angularPID.setPID(RobotMap.PIDMap.FOD_P, RobotMap.PIDMap.FOD_I, RobotMap.PIDMap.FOD_D);
		enableAngularControl();
	}
	
	//Disable F.O.D.
	public void disableFOD()
	{
		fieldOriented = false; 
		angularTarget = 0;
		angularPID.disable();
		disableAngularControl();
	}
	
	//Check F.O.D.
	public boolean fieldOriented()
	{
		return fieldOriented;
	}
	
	//return Drive Train angular target for F.O.D.
	public double angularTarget()
	{
		return angularTarget; 
	}
	
	//Set F.O.D. target angle
	public void setAngularTarget(double target)
	{
		angularTarget = target; 
	}
	
	//Enable Angular Correction in Drive train
	public void enableAngularControl()
	{
		angularPID.setSetpoint(angularTarget);
		angularPID.enable();
	}
	
	public void disableAngularControl()
	{
		whileRotating = false;
	}
	
	public void enableRotateTo()
	{
		whileRotating = true;
		PIDOutput = 0;
		angularPID.disable();
		
		angularPID.setPID(RobotMap.PIDMap.ANG_P, RobotMap.PIDMap.ANG_I, RobotMap.PIDMap.ANG_D);
		enableAngularControl();
	}
	
	public void enableRotateToV2()
	{
		whileRotating = true;
		PIDOutput = 0;
		angularPID.disable();
//		pod0.setDriveMaxVoltage(12, -12);
//		pod1.setDriveMaxVoltage(12, -12);
//		pod2.setDriveMaxVoltage(12, -12);
//		pod3.setDriveMaxVoltage(12, -12);
		
		angularPID.setOutputRange(-8500, 8500);
		angularPID.setPID(RobotMap.PIDMap.ANG_P, RobotMap.PIDMap.ANG_I, RobotMap.PIDMap.ANG_D);
//		angularPID.setSetpoint(angularTarget);
//		angularPID.enable();
		enableAngularControl();
	}
	
	public void disableRotateTo()
	{
		angularPID.disable();
		zeroWheelSpeeds();
		whileRotating = false;
	}
	
	public void disableRotateToV2()
	{
		whileRotating = false;
		angularPID.disable();
		zeroWheelSpeeds();
		angularPID.setOutputRange(-180, 180);
		angularPID.setPID(RobotMap.PIDMap.ANG_P, RobotMap.PIDMap.ANG_I, RobotMap.PIDMap.ANG_D);
		pod0.setDriveMaxVoltage(12, 0);
		pod1.setDriveMaxVoltage(12, 0);
		pod2.setDriveMaxVoltage(12, 0);
		pod3.setDriveMaxVoltage(12, 0);
	}
	
	public double getYaw()
	{
		return gyro.pidGet();
	}
	
	public void zeroWheelSpeeds()
	{
		pod0.setWheelSpeed(0);
		pod1.setWheelSpeed(0);
		pod2.setWheelSpeed(0);
		pod3.setWheelSpeed(0);
	}
	
	//Pass in a polar vector and angle the robot will move in that direction and rotation
	public void autoDrive(double mag, double headingTheta, double baseTheta)
	{
	/*	        /|
		      /  |  H*sin(theta) = Y
		    H    Y  H*cos(theta) = X
		  /      |
		/0___X___|   */
		
		targetX = mag * Math.cos( Math.toRadians(headingTheta) );
		targetY = mag * Math.sin( Math.toRadians(headingTheta) );
		calcWheelsFromRectCoords(targetX, targetY, baseTheta);
	}
	
	public void PIDDrive()
	{
		calcWheelsFromRectCoords(targetX, targetY, PIDOutput); 
//		if (Math.abs(correctionAngle) < 1 && fieldOriented)
//		{
//			calcWheelsFromRectCoords(targetX, targetY, 0);
//		}
//		else {
//			calcWheelsFromRectCoords(targetX, targetY, correctionAngle); 
//		}
	}
	
	public void toggleLowPowerMode(){
		if(lowPowerMode == false){
			lowPowerMode = true;
			
		}
		else if(lowPowerMode == true){
			lowPowerMode = false;
		}
	}
	public void PIDDriveV2()
	{			
		pod0.setSteeringAngle (45);
		pod1.setSteeringAngle (45+90);
		pod2.setSteeringAngle (-45-90);
		pod3.setSteeringAngle (-45);

		pod0.setWheelSpeed(PIDOutput);
		pod1.setWheelSpeed(PIDOutput);	
		pod2.setWheelSpeed(PIDOutput);
		pod3.setWheelSpeed(PIDOutput);
	}
	
	//Drive the Robot in teleop. two joystick input
	public void teleopDrive()
	{
		x      = -OI.strafeJoystick.getX();
		y      = -OI.strafeJoystick.getY();
		if(!fieldOriented)
		rotate = -OI.rotateJoystick.getX();
		
		//cubic ramping
		x = Math.pow(x, 3);
		y = Math.pow(y, 3);
		//rotate = Math.pow(rotate, 3);
		// low power mode <-----------------------------------TO CHANGE LOW POWERMODE MAKE MODIFICATIONS HERE
		if(lowPowerMode){
			if( x > joystickDeadband ){
				x = RobotMap.DriveMap.lowPowerSpeed;
			}
			else if( x < -joystickDeadband ){
				x = -RobotMap.DriveMap.lowPowerSpeed;
			}
			else{ 
				x = 0;
			}
			if( y > joystickDeadband ){
				y = RobotMap.DriveMap.lowPowerSpeed;
			}
			else if( y < -joystickDeadband ){
				y = -RobotMap.DriveMap.lowPowerSpeed;
			}
			else{ 
				y = 0;
			}
		}
		
		if(fieldOriented && !(Math.abs(-OI.rotateJoystick.getX()) < joystickDeadband))
		{
			fieldRotating = true;
		}
		
		if(fieldOriented && fieldRotating)
		{
			fieldOriented = false;
			disableFOD();
			rotate = -OI.rotateJoystick.getX();
		}
		
		if(fieldRotating && (Math.abs(-OI.rotateJoystick.getX()) < joystickDeadband))
		{
			fieldRotating = false;
			fieldOriented = true;
			enableFOD();
		}
		
		if (Math.abs(x) < joystickDeadband)
			x = 0;
		if (Math.abs(y) < joystickDeadband)
			y = 0;
		
		if (Math.abs(rotate) < joystickDeadband)
			rotate = 0;

		if (Math.abs(y) < joystickDeadband && Math.abs(x) < joystickDeadband && Math.abs(rotate) < joystickDeadband)
		{
			inDeadBand = true; 
		}
		
		else {
			inDeadBand = false;
			rotate = MathStuff.mapStickToAngle(rotate/20);
		}
		
		if (fieldOriented)
		{
			double theta = - gyro.getYaw();
			double[] vector = new double[2];
			
			vector[0] = x;
			vector[1] = y;
			
			vector = MathStuff.rotateVector(vector, theta);
			targetX = vector[0];
			targetY = vector[1];
			
			if(!(whileRotating || autoDriving))
			{
				PIDDrive();
			}
		}
		else {
			calcWheelsFromRectCoords(x, y, rotate);
		}
	}

	public void zeroSpeeds()
	{
		targetX = 0;
		targetY = 0;
		x = 0;
		y = 0;
	}
	
	public void autoDriveRectCoords(double x, double y, double rotate)
	{
		targetX = x;
		targetY = y;
		calcWheelsFromRectCoords(targetX, targetY, rotate);
	}

	public void calcWheelsFromRectCoords(double x, double y, double rotate)
	{
		//calculate angle/speed setpoints using 30x30 in. robot
		double L = 23.5, W = 28.5;
		double R = Math.sqrt((L * L) * (W * W));
		double A = x - rotate * (L / R);
		double B = x + rotate * (L / R);
		double C = y - rotate * (W / R);
		double D = y + rotate * (W / R);
		
		//find wheel speeds		
		double pod1WheelSpeed = Math.sqrt((B * B) + (C * C));  // 2 FRONT RIGHT
		double pod0WheelSpeed = Math.sqrt((B * B) + (D * D));  // 1 FRONT LEFT
		double pod3WheelSpeed = Math.sqrt((A * A) + (D * D));  // 0 REAR LEFT
		double pod2WheelSpeed = Math.sqrt((A * A) + (C * C));  // 3 REAR RIGHT

		//normalize wheel speeds
		double max = pod3WheelSpeed;
		if(pod1WheelSpeed > max)
		{
			max = pod1WheelSpeed;
		}
		if(pod2WheelSpeed > max)
		{
			max = pod2WheelSpeed;
		}
		if(pod0WheelSpeed > max)
		{
			max = pod0WheelSpeed;
		}
		if(max > 1)
		{
			//Max is more than 1.0, so normalize.
			pod0WheelSpeed /= max;
			pod1WheelSpeed /= max;
			pod2WheelSpeed /= max;
			pod3WheelSpeed /= max;
		}
		pod0WheelSpeed *= RobotMap.DriveMap.maxSpeed;
		pod1WheelSpeed *= RobotMap.DriveMap.maxSpeed;
		pod2WheelSpeed *= RobotMap.DriveMap.maxSpeed;
		pod3WheelSpeed *= RobotMap.DriveMap.maxSpeed;

		//find steering angles
		//controls to keep wheels from turning when stick returns to zero
		/*if (!inDeadBand || autoDriving || whileRotating)
		{
			double pod1SteeringAngle = Math.toDegrees(Math.atan2(B, C)); //FRONT RIGHT
			double pod0SteeringAngle = Math.toDegrees(Math.atan2(B, D)); //FRONT LEFT 
			double pod3SteeringAngle = Math.toDegrees(Math.atan2(A, D)); //REAR LEFT 
			double pod2SteeringAngle = Math.toDegrees(Math.atan2(A, C)); //REAR RIGHT
			pod0.setSteeringAngle (pod0SteeringAngle);
			pod1.setSteeringAngle (pod1SteeringAngle);
			pod2.setSteeringAngle (pod2SteeringAngle);
			pod3.setSteeringAngle (pod3SteeringAngle);
		}
		if (!inDeadBand || autoDriving || whileRotating ) {
			pod0.setWheelSpeed(pod0WheelSpeed);
			pod1.setWheelSpeed(pod1WheelSpeed);	
			pod2.setWheelSpeed(pod2WheelSpeed);
			pod3.setWheelSpeed(pod3WheelSpeed);
		} else {
			pod0.setWheelSpeed(0);
			pod1.setWheelSpeed(0);	
			pod2.setWheelSpeed(0);
			pod3.setWheelSpeed(0);
		}*/
		if (!inDeadBand || autoDriving || whileRotating)
		{
			double pod1SteeringAngle = Math.toDegrees(Math.atan2(B, C)); //FRONT RIGHT
			double pod0SteeringAngle = Math.toDegrees(Math.atan2(B, D)); //FRONT LEFT 
			double pod3SteeringAngle = Math.toDegrees(Math.atan2(A, D)); //REAR LEFT 
			double pod2SteeringAngle = Math.toDegrees(Math.atan2(A, C)); //REAR RIGHT
			pod0.setPodSpeedAndAngle(pod0SteeringAngle, pod0WheelSpeed);
			pod1.setPodSpeedAndAngle(pod1SteeringAngle, pod1WheelSpeed);
			pod2.setPodSpeedAndAngle(pod2SteeringAngle, pod2WheelSpeed);
			pod3.setPodSpeedAndAngle(pod3SteeringAngle, pod3WheelSpeed);
			
		} else {
			pod0.setWheelSpeed(0);
			pod1.setWheelSpeed(0);	
			pod2.setWheelSpeed(0);
			pod3.setWheelSpeed(0);
		}
	}
	
	public void testPodClosedLoop(int pod, double speed, double angle)
	{
		if (pod == 0)
		{
			pod0.setSteeringAngle(angle);
			pod0.setWheelSpeed(speed);
		}
		else if (pod == 1)
		{
			pod1.setSteeringAngle(angle);
			pod1.setWheelSpeed(speed);
		}
		else if (pod == 2)
		{
			pod2.setSteeringAngle(angle);
			pod2.setWheelSpeed(speed);
		}
		else if (pod == 3)
		{
			pod3.setSteeringAngle(angle);
			pod3.setWheelSpeed(speed);
		}
	}
	
	public void testPodPercentVBus(int pod, double speed, double angle)
	{
		if (pod == 0)
		{
			pod0.setPercentVBusSteer(angle);
			pod0.setPercentVBusDrive(speed);
		}
		else if (pod == 1)
		{
			pod1.setPercentVBusSteer(angle);
			pod1.setPercentVBusDrive(speed);
		}
		else if (pod == 2)
		{
			pod2.setPercentVBusSteer(angle);
			pod2.setPercentVBusDrive(speed);
		}
		else if (pod == 3)
		{
			pod3.setPercentVBusSteer(angle);
			pod3.setPercentVBusDrive(speed);
		}
	}
	
	public void post()
	{
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
		SmartDashboard.putNumber("DriveTrain/Correction Angle:", PIDOutput);
		SmartDashboard.putBoolean("Digital Input", digitalIn.get());
		SmartDashboard.putNumber("Analog Input", analogIn.getAverageVoltage());
		SmartDashboard.putNumber("DriveTrain/TargetX", targetX);
		SmartDashboard.putNumber("DriveTrain/TargetY", targetY);
		SmartDashboard.putBoolean("DriveTrain/Auto Driving", autoDriving);
		SmartDashboard.putBoolean("DriveTrain/Auto Rotating", whileRotating);
		SmartDashboard.putBoolean("DriveTrain/Field Rotating", fieldRotating);
	}
	
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
	@Override
	public void pidWrite(double output)
	{
		PIDOutput = -output;
	}
	
	public void zeroGyro()
	{
		gyro.zeroYaw();
	}
	public double getAnalog() {
		return analogIn.getAverageVoltage(); 
	}
	public boolean getDigital()
	{
		return digitalIn.get();
	}
}

