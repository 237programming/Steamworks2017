package org.usfirst.frc.team237.robot.commands;

import org.usfirst.frc.team237.robot.Robot;
import org.usfirst.frc.team237.robot.subsystems.RopeSubsystem.Speed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class MoveRopeGrabber extends Command {
	
	double seconds;
	
	Timer time;

    public MoveRopeGrabber(double seconds) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.ropeSubsystem);
    	this.seconds = seconds;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	time = new Timer();
    	time.start();
    	Robot.ropeSubsystem.setSpeed(Speed.Slow);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return time.get() > seconds;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.ropeSubsystem.setSpeed(Speed.Off);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.ropeSubsystem.setSpeed(Speed.Off);
    }
}
