package org.usfirst.frc.team237.robot.commands.autonomous;

import org.usfirst.frc.team237.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AlignToGear extends Command {
	private double targetLocation; 
	private boolean targetVisible; 
	private boolean onTarget; 
    public AlignToGear() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	targetLocation = Robot.driveTrain.getAnalog(); 
    	targetVisible = Robot.driveTrain.getDigital(); 
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (targetLocation < 510) {
    		Robot.driveTrain.autoDrive(0.1, 180.0, 0);
    	}
    	else if ( targetLocation > 514) { 
    		Robot.driveTrain.autoDrive(0.1, 0, 0);
    	} 
    	else {
    		onTarget = true; 
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (onTarget || !targetVisible);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.autoDrive(0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.autoDrive(0, 0, 0);
    }
}
