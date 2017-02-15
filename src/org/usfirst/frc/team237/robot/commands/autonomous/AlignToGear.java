package org.usfirst.frc.team237.robot.commands.autonomous;

import org.usfirst.frc.team237.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AlignToGear extends Command {
	
	private double targetLocation; 
	private boolean targetVisible; 
	private boolean onTarget;
	private double seconds;
	
	Timer time;
	
    public AlignToGear(float seconds) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	time = new Timer();
    	this.seconds = seconds;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.lightOn();
    	targetLocation = Robot.driveTrain.getAnalog(); 
    	targetVisible = Robot.driveTrain.getDigital();
    	time.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(time.get() > seconds)
    	{
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
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (onTarget || !targetVisible);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.lightOff();
    	Robot.driveTrain.autoDrive(0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.lightOff();
    	Robot.driveTrain.autoDrive(0, 0, 0);
    }
}
