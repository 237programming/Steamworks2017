package org.usfirst.frc.team237.robot.commands;

import org.usfirst.frc.team237.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RotateTo extends Command {

	double angle;
	
    public RotateTo(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.setAngularTarget(angle);
    	Robot.driveTrain.enableRotateTo();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Math.abs(Robot.driveTrain.getYaw() - Robot.driveTrain.angularTarget()) <= 2;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.disableRotateTo();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.disableRotateTo();
    }
}
