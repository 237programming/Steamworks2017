package org.usfirst.frc.team237.robot.commands;

import org.usfirst.frc.team237.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SmartRotateTo extends Command {

	double angle;
	int counter = 0;
	boolean doneRotating = false;
	double deadband = 5;
	
    public SmartRotateTo(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.setAngularTarget(angle);
    	Robot.driveTrain.enableRotateToV2();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.PIDDriveV2();
    	if(Math.abs(Robot.driveTrain.getYaw() - Robot.driveTrain.angularTarget()) <= deadband) counter++;
    	else if(!(Math.abs(Robot.driveTrain.getYaw() - Robot.driveTrain.angularTarget()) <= deadband)) counter = 0;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return counter >= 5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.disableRotateToV2();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.disableRotateToV2();
    }
}
