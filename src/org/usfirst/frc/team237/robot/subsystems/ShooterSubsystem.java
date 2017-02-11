package org.usfirst.frc.team237.robot.subsystems;

import org.usfirst.frc.team237.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterSubsystem extends Subsystem {
	
	CANTalon shooterTalon;
	CANTalon feederTalon;
	
	Relay light = new Relay(0);
	
	private double targetSpeed;
	
	public ShooterSubsystem()
	{
		shooterTalon = new CANTalon(RobotMap.ShooterMap.shooterTalon);
		feederTalon  = new CANTalon(RobotMap.ShooterMap.feederTalon);
		shooterTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooterTalon.changeControlMode(TalonControlMode.Speed);
		shooterTalon.configNominalOutputVoltage(+0.0, -0.0);
		shooterTalon.configPeakOutputVoltage(+12.0, -12.0);
		shooterTalon.setProfile(0);
		shooterTalon.setP(0.2);
		shooterTalon.setI(0.0);
		shooterTalon.setD(0.0);
		shooterTalon.setF(0.0);
		feederTalon.changeControlMode(TalonControlMode.PercentVbus);
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
	
	public void setShooter(double speed)
	{
		targetSpeed = speed;
		shooterTalon.set(speed);
	}
	
	public double shooterSpeed()
	{
		return shooterTalon.get();
	}
	
	public void feederIn()
	{
		feederTalon.set(1.0);
	}
	
	public void feederOut()
	{
		feederTalon.set(-1.0);
	}
	
	public void feederOff()
	{
		feederTalon.set(0.0);
	}
	
	public boolean upToSpeed(double error)
	{
		return shooterSpeed() < targetSpeed + error &&
			   shooterSpeed() > targetSpeed - error;
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

