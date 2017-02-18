package org.usfirst.frc.team237.robot.subsystems;

import org.usfirst.frc.team237.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeSubsystem extends Subsystem { 
	public CANTalon intakeTalon; 
	public IntakeSubsystem(){
		intakeTalon = new CANTalon(RobotMap.ShooterMap.intakeTalon); 
		intakeTalon.changeControlMode(TalonControlMode.PercentVbus);
	}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public void intakeIn(double speed) {
		intakeTalon.set(speed);
	}
	
	public void intakeOut() {
		intakeTalon.set(-1.0);
	}
	
	public void intakeOff() {
		intakeTalon.set(0.0);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

