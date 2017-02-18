package org.usfirst.frc.team237.robot.commands.autonomous;

import org.usfirst.frc.team237.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AlignToGear extends Command {
	
	private double targetLocation = 0; 
	private boolean targetVisible = false; 
	private boolean onTarget = false;
	
    public AlignToGear() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	targetLocation = Robot.driveTrain.getAnalog();
    	targetVisible = Robot.driveTrain.getDigital();
    	if(!targetVisible) onTarget = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	targetLocation = Robot.driveTrain.getAnalog();
    	targetVisible = Robot.driveTrain.getDigital();
    	
    	if (targetLocation < 2.05) {
//    		Robot.driveTrain.autoDrive(0.1, 45, 0);
    		Robot.driveTrain.calcWheelsFromRectCoords(-1, 0, 0);
    	}
    	else if ( targetLocation > 2.25) { 
//    		Robot.driveTrain.autoDrive(0.1, -45, 0);
    		Robot.driveTrain.calcWheelsFromRectCoords(1, 0, 0);
    	} 
    	else {
    		onTarget = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return onTarget;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("2.05 < " + Robot.driveTrain.getAnalog() + " < 2.25");
    	Robot.driveTrain.autoDrive(0, 0, 0);
    	onTarget = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.autoDrive(0, 0, 0);
    	onTarget = false;
    }
}
