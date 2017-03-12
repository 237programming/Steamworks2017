package org.usfirst.frc.team237.robot.commands;

import org.usfirst.frc.team237.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;



public class RotateTo extends Command {

	double angle;
	int counter = 0;
	boolean doneRotating = false;
	double deadband = 7;
	
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
    	Robot.driveTrain.PIDDrive();
    	if(Math.abs(Robot.driveTrain.getYaw() - Robot.driveTrain.angularTarget()) <= deadband) counter++;
    	else if(!(Math.abs(Robot.driveTrain.getYaw() - Robot.driveTrain.angularTarget()) <= deadband)) counter = 0;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(counter >= 5) return true;
    	else return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("DONE ROTATING AT " + Robot.driveTrain.getYaw());
    	Robot.driveTrain.disableRotateTo();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.disableRotateTo();
    }
}
