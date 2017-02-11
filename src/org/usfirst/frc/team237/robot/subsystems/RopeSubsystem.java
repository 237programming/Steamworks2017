package org.usfirst.frc.team237.robot.subsystems;

import org.usfirst.frc.team237.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class RopeSubsystem extends Subsystem {

	public CANTalon hangerTalon;
	
	public enum Speed
	{
		Fast,
		Slow,
		Off
	};
	
	public Speed speed;
	
	public RopeSubsystem()
	{
		hangerTalon = new CANTalon(RobotMap.ShooterMap.hangerTalon);
		hangerTalon.changeControlMode(TalonControlMode.PercentVbus);
	}
	
	public void setSpeed(Speed speed)
	{
		if(speed == Speed.Off)  hangerTalon.set(0.0);
		if(speed == Speed.Slow) hangerTalon.set(0.5);
		if(speed == Speed.Fast) hangerTalon.set(1.0);
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

