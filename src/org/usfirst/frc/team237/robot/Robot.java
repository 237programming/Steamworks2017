package org.usfirst.frc.team237.robot;

import org.usfirst.frc.team237.robot.commands.autonomous.BlueLeftGear;
import org.usfirst.frc.team237.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team237.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team237.robot.subsystems.RopeSubsystem;
import org.usfirst.frc.team237.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static OI oi;
	public static DriveSubsystem driveTrain         = new DriveSubsystem();
	public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	public static IntakeSubsystem intakeSubsystem   = new IntakeSubsystem();
	public static RopeSubsystem ropeSubsystem       = new RopeSubsystem();
	
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		chooser.addDefault("Default Auto", autonomousCommand);
		SmartDashboard.putData("Auto mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.post();
		shooterSubsystem.post();
		
/*		String autoSelected = SmartDashboard.getString("SmartDashboard/AutoCommand");
		autoSelected = autoSelected.toLowerCase();
		switch(autoSelected)
		{
			case "Blue Left":
				
			break;
				
			case "Blue Center":
				
			break;
				
			case "Blue Right":
				
			break;
				
			case "Red Left":
				
			break;
				
			case "Red Center":
				
			break;
				
			case "Red Right":
				
			break;
		}
*/
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		driveTrain.lightOn();
		autonomousCommand = new BlueLeftGear();
		
		/*
		String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		
		switch(autoSelected) {
			case "My Auto":
				autonomousCommand = new MyAutoCommand();
				break;
			case "Default Auto":
				default:
					autonomousCommand = new ExampleCommand();
					break;
		}
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.post();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		driveTrain.disableFOD();
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		Scheduler.getInstance().run();
		driveTrain.teleopDrive();
		
		driveTrain.post();
		shooterSubsystem.post();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
