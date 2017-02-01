package org.usfirst.frc.team237.robot.commands;

import org.usfirst.frc.team237.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveForTimeAtSpeed extends Command {
	double seconds;
	double speed;
	double angle;
	
	Timer time;
	
	public DriveForTimeAtSpeed(double seconds, double speed, double angle) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
		
		this.seconds = seconds;
		this.speed = speed;
		this.angle = angle;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		time = new Timer();
		time.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.driveTrain.autoDrive(speed, angle);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return time.get() > seconds;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.driveTrain.autoDrive(0, angle);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.driveTrain.autoDrive(0, angle);
	}
}
