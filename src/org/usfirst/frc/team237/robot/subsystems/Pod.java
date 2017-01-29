package org.usfirst.frc.team237.robot.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.*;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import org.usfirst.frc.team237.robot.MathStuff;
import org.usfirst.frc.team237.robot.RobotMap;
/**
 *
 */
public class Pod extends Subsystem {
	public CANTalon drive;
	public CANTalon steer; 
	public double targetPosition=0.0; 
	public double targetSpeed=0.0; 
	public int podNumber; 
	private int offset; 
	private PIDController steerPID; 
	public Pod(int driveTalon, int steeringTalon, int podNumber){	
		drive = new CANTalon(driveTalon);
		steer = new CANTalon(steeringTalon);
		steer.setFeedbackDevice(FeedbackDevice.AnalogPot);
		steerPID = new PIDController(.2,0,0,steer,steer);
		steer.reverseSensor(false);
//		steer.setP(12.0);
//		steer.setI(0);
//		steer.setD(0);
//		steer.setF(0);
//		steer.setProfile(0);
		steer.configNominalOutputVoltage(+ 0.0, - 0.0);
		steer.configPeakOutputVoltage(+ 12f, - 12f);
		steer.setAllowableClosedLoopErr(0);
		steerPID.setContinuous();
		drive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		drive.configNominalOutputVoltage(+ 0.0, - 0.0);
		drive.configPeakOutputVoltage(+ 12.0, - 0.0);
		drive.setProfile(0);
		drive.setP(0.20);
		drive.setI(0.002);
		drive.setD(6.0);
		drive.setF(0.11);
		drive.reverseSensor(false);
		this.podNumber = podNumber;
		this.offset = 0; 
		zeroSensorsAndThrottle(); 
		enableClosedLoopAngle();
		enableClosedLoopSpeed(); 
		
	}
	public Pod(int driveTalon, int steeringTalon, int podNumber, int offset){	
		drive = new CANTalon(driveTalon);
		steer = new CANTalon(steeringTalon);
		steerPID = new PIDController(0.005,0,0,steer,steer);
		steer.setFeedbackDevice(FeedbackDevice.AnalogPot);
		steerPID.setInputRange(0, 1023);
		steerPID.setOutputRange(-1.0, 1.0);
		steerPID.setContinuous();
		steer.reverseSensor(false);
		steer.configNominalOutputVoltage(+ 0.0, - 0.0);
		steer.configPeakOutputVoltage(+ 12f, - 12f);
		drive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		drive.configNominalOutputVoltage(+ 0.0, - 0.0);
		drive.configPeakOutputVoltage(+ 12.0, - 4.0);
		drive.setProfile(0);
		drive.setP(0.20);
		drive.setI(0.002);
		drive.setD(6.0);
		drive.setF(0.11);
		drive.setAllowableClosedLoopErr(200);
		drive.reverseSensor(false);
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
		
	
	}
	public void enableClosedLoopSpeed(){
		drive.setVoltageRampRate(0);
		drive.changeControlMode(TalonControlMode.Speed);
		drive.set(targetSpeed);
		
	}
	public void enableClosedLoopAngle(){
		steerPID.enable();
		steer.changeControlMode(TalonControlMode.PercentVbus);
		double setPoint = MathStuff.mapAngleToEnc(targetPosition)+offset;
		if (setPoint > 1023)
		{
			setPoint -= 1024; 
		} else if (setPoint < 0) {
			setPoint += 1024;
		}
		steerPID.setSetpoint(setPoint);
	
	}
    public void setSteeringAngle(double angle) {
    	targetPosition = angle;
    	enableClosedLoopAngle();
    	post(); 
    	
    }
	public void setWheelSpeed(double speed){
		targetSpeed = speed;
		enableClosedLoopSpeed();
		post();
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
    	
    	//SmartDashboard.putNumber("Pod" + podNumber + "/Steer/P",steer.getP() );
    	//SmartDashboard.putNumber("Pod" + podNumber + "/Steer/I",steer.getI() );
    	//SmartDashboard.putNumber("Pod" + podNumber + "/Steer/D",steer.getD() );
    	//SmartDashboard.putNumber("Pod" + podNumber + "/Steer/F",steer.getF() );
	}
	public int getOffSet() {
		return offset;
				
	}
	public void setOffSet(int offset) {
		this.offset = offset;
				
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
@Override
protected void initDefaultCommand() {
	// TODO Auto-generated method stub
	
}
}