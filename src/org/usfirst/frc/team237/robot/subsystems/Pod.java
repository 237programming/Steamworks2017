package org.usfirst.frc.team237.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.*;
/**
 *
 */
public class Pod extends Subsystem {
	public CANTalon drive;
	public CANTalon steer; 

public Pod(int drivePWM, int steeringPWM, int podNumber){	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

