package org.usfirst.frc.team237.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.*;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
/**
 *
 */
public class Pod extends Subsystem {
	public CANTalon drive;
	public CANTalon steer; 
	public double targetPosition; 
	public Pod(int driveTalon, int steeringTalon, int podNumber){	
		drive = new CANTalon(driveTalon);
		steer = new CANTalon(steeringTalon);
		SmartDashboard.putNumber("Steering Pod", podNumber);
		steer.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		steer.reverseSensor(false);
		steer.setP(0.2);
		steer.setI(0);
		steer.setD(0);
		steer.setF(0);
		steer.setProfile(0);
		steer.configNominalOutputVoltage(+ 0.0, - 0.0);
		steer.configPeakOutputVoltage(+ 3f, - 3f);
		steer.setAllowableClosedLoopErr(0);
		zeroSensorsAndThrottle(); 
		
	}
	public void zeroSensorsAndThrottle(){
		steer.setPosition(0);
		targetPosition = 0; 
		steer.changeControlMode(TalonControlMode.PercentVbus);
		steer.set(0);
	
	}
	public void enableClosedLoop(){
		steer.setVoltageRampRate(0);
		steer.changeControlMode(TalonControlMode.Position);
		steer.set(targetPosition);
		
	}
    public void setSteeringAngle(double angle) {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }


	public void setWheelSpeed(double speed){
		
	}

	public void intiDefaultCommand() {
	}

@Override
protected void initDefaultCommand() {
	// TODO Auto-generated method stub
	
}
}