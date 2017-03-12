package org.usfirst.frc.team237.robot;

import org.usfirst.frc.team237.robot.commands.DriveForCountsAtSpeed;
import org.usfirst.frc.team237.robot.commands.autonomous.LeftGear;
import org.usfirst.frc.team237.robot.commands.autonomous.RightGear;
import org.usfirst.frc.team237.robot.commands.autonomous.CenterGearGroup;
import org.usfirst.frc.team237.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team237.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team237.robot.subsystems.RopeSubsystem;
import org.usfirst.frc.team237.robot.subsystems.ShooterSubsystem;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static OI oi;
	public static DriveSubsystem driveTrain         = new DriveSubsystem();
	public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	public static IntakeSubsystem intakeSubsystem   = new IntakeSubsystem();
	public static RopeSubsystem ropeSubsystem       = new RopeSubsystem();
	
	Command autonomousCommand;
	SendableChooser<Command> chooser;
	
	double time = 0, speed = 0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		chooser = new SendableChooser<Command>();
		chooser.addDefault("Default Auto", new LeftGear());
		chooser.addObject("Blue Left", new LeftGear());
		chooser.addObject("Blue Center", new CenterGearGroup());
		chooser.addObject("Blue Right", new RightGear());
		chooser.addObject("Red Center", new CenterGearGroup());
		SmartDashboard.putData("Auto mode", chooser);
		UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
		cam.setResolution(1280, 720);
		
		//CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 1920, 1080);
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
		time = SmartDashboard.getNumber("Time", 0.1);
		speed = SmartDashboard.getNumber("Speed", 0.1);
		
//		String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
//		
//		switch(autoSelected) {
//			case "Blue Left":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Blue Left");
//				autonomousCommand = new BlueLeftGear();
//			break;
//				
//			case "Blue Center":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Blue Center");
//				autonomousCommand = new CenterGearGroup();
//			break;
//				
//			case "Blue Right":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Blue Right");
//				autonomousCommand = new BlueRightGear();
//			break;
//				
//			case "Red Left":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Red Left");
//				autonomousCommand = new BlueLeftGear();
//			break;
//				
//			case "Red Center":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Red Center");
//				autonomousCommand = new CenterGearGroup();
//			break;
//				
//			case "Red Right":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Red Right");
//				autonomousCommand = new BlueRightGear();
//			break;
//			default:
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Do Nothing");
//				autonomousCommand = new DriveForTimeAtSpeed(0, 0, 0, 0);
//		}

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
		driveTrain.zeroGyro();
		autonomousCommand = (Command) chooser.getSelected();
		if(autonomousCommand != null) autonomousCommand.start();
		
//		String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
//		
//		switch(autoSelected) {
//				case "Blue Left":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Blue Left");
//				autonomousCommand = new BlueLeftGear();
//			break;
//				
//			case "Blue Center":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Blue Center");
//				autonomousCommand = new CenterGearGroup();
//			break;
//				
//			case "Blue Right":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Blue Right");
//				autonomousCommand = new BlueRightGear();
//			break;
//				
//			case "Red Left":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Red Left");
//				autonomousCommand = new BlueLeftGear();
//			break;
//				
//			case "Red Center":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Red Center");
//				autonomousCommand = new CenterGearGroup();
//			break;
//				
//			case "Red Right":
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Red Right");
//				autonomousCommand = new BlueRightGear();
//			break;
//			default:
//				SmartDashboard.putString("SmartDashboard/AutoCommand/String 5", "Do Nothing");
//				autonomousCommand = new DriveForTimeAtSpeed(0, 0, 0, 0);
//		}

		// schedule the autonomous command (example)
//		if (autonomousCommand != null)
//			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.post();
		shooterSubsystem.post();
		
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if(OI.toggleFOD.get()) driveTrain.enableFOD();
		else driveTrain.disableFOD();
		shooterSubsystem.setShooter(0);
		intakeSubsystem.intakeOff();
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		driveTrain.lightOn();
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
