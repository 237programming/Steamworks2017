package org.usfirst.frc.team237.robot.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.*;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import org.usfirst.frc.team237.robot.MathStuff;

public class PodSubsystem extends Subsystem {
	public CANTalon drive;
	public CANTalon steer; 
	public double targetPosition=0.0; 
	public double targetSpeed=0.0; 
	public int podNumber; 
	private int offset; 
	private PIDController steerPID; 
	private boolean enableSmartSteer = false; 
	public PodSubsystem(int driveTalon, int steeringTalon, int podNumber){	
//		Pod constructor without offset
		drive = new CANTalon(driveTalon);
		steer = new CANTalon(steeringTalon);
		steerPID = new PIDController(.2,0,0,steer,steer);
		steerPID.setContinuous();
//		Declare Talons and PIDCont. 
		steer.setFeedbackDevice(FeedbackDevice.AnalogPot);
		drive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
//		Math for feedback device
		steer.reverseSensor(false);		
		drive.reverseSensor(false);
		steer.configNominalOutputVoltage(+ 0.0, - 0.0);
		steer.configPeakOutputVoltage(+ 12f, - 12f);
//		Config voltage
		steer.setAllowableClosedLoopErr(0);
		drive.configNominalOutputVoltage(+ 0.0, - 0.0);
		drive.configPeakOutputVoltage(+ 12.0, - 12.0);
		drive.setProfile(0);
		drive.setP(0.20);
		drive.setI(0.002);
		drive.setD(6.0);
		drive.setF(0.11);
//		Set drive values 
		this.podNumber = podNumber;
		this.offset = 0; 
		zeroSensorsAndThrottle(); 
		enableClosedLoopAngle();
		enableClosedLoopSpeed(); 
	}
	public PodSubsystem(int driveTalon, int steeringTalon, int podNumber, int offset){	
//		Pod constructor with offset
		drive = new CANTalon(driveTalon);
		steer = new CANTalon(steeringTalon);
		steerPID = new PIDController(0.005,0,0,steer,steer);
		steerPID.setInputRange(0, 1023);
		steerPID.setOutputRange(-1.0, 1.0);
		steerPID.setContinuous();
//		Ranges
		steer.setFeedbackDevice(FeedbackDevice.AnalogPot);
		drive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		steer.reverseSensor(false);
		drive.reverseSensor(false);
		steer.configNominalOutputVoltage(+ 0.0, - 0.0);
		steer.configPeakOutputVoltage(+ 12f, - 12f);
		drive.configNominalOutputVoltage(+ 0.0, - 0.0);
		drive.configPeakOutputVoltage(+ 12.0, - 0.0);
		drive.setProfile(0);
		drive.setP(0.20);
		drive.setI(0.002);
		drive.setD(6.0);
		drive.setF(0.11);
		drive.setAllowableClosedLoopErr(200);
		this.podNumber = podNumber;
		this.offset = offset; 
		zeroSensorsAndThrottle(); 
		enableClosedLoopAngle();
		enableClosedLoopSpeed(); 
	}
	public void zeroSensorsAndThrottle(){
		steer.setPosition(0);
		targetPosition = 0; 
		steer.changeControlMode(TalonControlMode.PercentVbus);
		steer.set(0);
		drive.setPosition(0);
		targetSpeed = 0;
		drive.changeControlMode(TalonControlMode.PercentVbus);
		drive.set(0);
//		Set Talons and positions to 0
	}
	public void enableClosedLoopSpeed(){
		drive.setVoltageRampRate(0);
		drive.changeControlMode(TalonControlMode.Speed);
		drive.clearIAccum();
		drive.set(targetSpeed);
//		Enable drive to target speed from Talon
	}
	public void enableClosedLoopAngle(){
		steerPID.enable();
		steer.changeControlMode(TalonControlMode.PercentVbus);
		double setPoint = MathStuff.mapAngleToEnc(targetPosition)+offset;
		setPoint = MathStuff.normalizeEncInput(setPoint);
		steerPID.setSetpoint(setPoint);
//		Enable angle PID and math
	}
    public void setSteeringAngle(double angle) {
    	targetPosition = angle;
    	enableClosedLoopAngle();
    	post(); 
//		Set angle using enableClosedLoopAngle
    }
	public void setWheelSpeed(double speed){
		targetSpeed = speed;
		enableClosedLoopSpeed();
		//post();
//		Set speed using enableClosedLoopSpeed
	}
	public void intiDefaultCommand() {	
	}
	public void post(){
		SmartDashboard.putNumber("Pod" + podNumber + "/Drive Closed Loop Error", drive.getClosedLoopError());
		SmartDashboard.putNumber("Pod" + podNumber + "/Steer Closed Loop Error", steer.getClosedLoopError());
		SmartDashboard.putNumber("Pod" + podNumber + "/Motor Speed", drive.getSpeed());
		SmartDashboard.putNumber("Pod" + podNumber + "/Target Speed", targetSpeed);
		SmartDashboard.putNumber("Pod" + podNumber + "/Steering Encoder", steer.getAnalogInRaw());
		SmartDashboard.putNumber("Pod" + podNumber + "/Current PID Output", steerPID.get());
    	SmartDashboard.putNumber("Pod" + podNumber + "/Tagret Encoder", targetPosition);
    	SmartDashboard.putNumber("Pod" + podNumber + "/Drive/P",drive.getP() );
    	SmartDashboard.putNumber("Pod" + podNumber + "/Drive/I",drive.getI() );
    	SmartDashboard.putNumber("Pod" + podNumber + "/Drive/D",drive.getD() );
    	SmartDashboard.putNumber("Pod" + podNumber + "/Drive/F",drive.getF() );
    	SmartDashboard.putNumber("Pod" + podNumber + "/Drive/ Encoder", drive.getEncPosition());
//		Give SmartDash data
	}
	
	public void setDriveMaxVoltage(double forward, double reverse)
	{
		drive.configPeakOutputVoltage(forward, reverse);
	}
	
	public int getOffSet() {
		return offset;		
	}
	public void setOffSet(int offset) {
		this.offset = offset;			
	}
	public int getEncPos(){
		return drive.getEncPosition();
	}
	public void zeroEnc(){
		drive.setEncPosition(0);
		drive.setEncPosition(0);
	}
	public void setPercentVBusSteer(double val){
		steer.changeControlMode(TalonControlMode.PercentVbus);
		steer.set(val);
	}
	public void setPercentVBusDrive(double val){
		drive.changeControlMode(TalonControlMode.PercentVbus);
		drive.set(val);
	}
	public void setSteerPID(double p, double i, double d)
	{
		steerPID.setPID(p, i, d);
	}
	public void setDrivePID(double p, double i, double d)
	{
		drive.setPID(p, i, d);
	}
	// enables the flag which allows the pod to minimize turning. 
	public void enableSmartSteer()
	{
		drive.configPeakOutputVoltage(12.0, -12.0);
		enableSmartSteer = true; 
	}
	// disables the flag which allows the pod to minimize turning. 
	public void disableSmartSteer()
	{
		drive.configPeakOutputVoltage(12.0, 0);
		enableSmartSteer = true;
	}
	/* This function will set the pods angle and speed. 
	 * If the pod has smart steering enabled it 
	 * will try and minimize the distance traveled during a rotation */ 
	public void setPodSpeedAndAngle(double targetAngle, double targetSpeed)
	{
		if (enableSmartSteer == true) 
		{
			double currentAngle = steerPID.get();
			boolean flipSpeed = false; 
			double difference =  targetAngle - currentAngle; 
			if ( Math.abs(difference) > 90.0)
			{
				targetAngle -= 180.0;
				flipSpeed = true; 
			}
			if (targetAngle < -180.0)
			{
				targetAngle += 360; 
			} else if ( targetAngle > 180.0 ) 
			{
				targetAngle -= 360; 
			}
			if ( flipSpeed == true )
			{
				targetSpeed = -targetSpeed; 
			}
		}
		setSteeringAngle(targetAngle); 
		setWheelSpeed(targetSpeed); 
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}
}