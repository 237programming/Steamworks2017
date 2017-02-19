package org.usfirst.frc.team237.robot.commands.autonomous;

import org.usfirst.frc.team237.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AlignToGear extends Command {
	
	private double headingTheta;
	private double targetLocation; 
	private boolean targetVisible;
	private boolean onTarget;
	private double min, max;
	
    public AlignToGear(double min, double max) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	this.min = min;
    	this.max = max;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.enableFOD();
    	headingTheta = 0;
    	targetLocation = 0;
    	targetVisible = false;
    	onTarget = false;
    	targetLocation = Robot.driveTrain.getAnalog();
    	targetVisible = Robot.driveTrain.getDigital();
		Robot.driveTrain.zeroSpeeds();
		Robot.driveTrain.autoDriving = true;
    	if(!targetVisible) onTarget = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	targetLocation = Robot.driveTrain.getAnalog();
    	targetVisible = Robot.driveTrain.getDigital();
    	
    	if(targetVisible)
    	{
			if (targetLocation < min) {
		//    		Robot.driveTrain.autoDrive(0.1, 45, 0);
				headingTheta = 0;
				Robot.driveTrain.autoDrive(.08, headingTheta, 0);
			}
			else if ( targetLocation > max) { 
		//    		Robot.driveTrain.autoDrive(0.1, -45, 0);
				headingTheta = 180;
				Robot.driveTrain.autoDrive(.08, headingTheta, 0);
			} 
			else {
				onTarget = true;
			}
    	} else {
    		targetVisible = false;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return onTarget || !targetVisible;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("--OnTarget:      " + onTarget + "\n" + 
    					   "--TargetVisible: " + targetVisible);
    	Robot.driveTrain.autoDrive(0, headingTheta, 0);
    	Robot.driveTrain.zeroSpeeds();
    	Robot.driveTrain.autoDriving = false;
    	onTarget = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	onTarget = false;
    	Robot.driveTrain.autoDrive(0, headingTheta, 0);
    	Robot.driveTrain.zeroSpeeds();
    	Robot.driveTrain.autoDriving = false;
    }
}
