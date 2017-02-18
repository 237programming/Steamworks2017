package org.usfirst.frc.team237.robot;

import org.usfirst.frc.team237.robot.commands.DriveForTimeAtSpeed;
import org.usfirst.frc.team237.robot.commands.IntakeCommand;
import org.usfirst.frc.team237.robot.commands.IntakeOffCommand;
import org.usfirst.frc.team237.robot.commands.OuttakeCommand;
import org.usfirst.frc.team237.robot.commands.PullRopeCommand;
import org.usfirst.frc.team237.robot.commands.ReadyShooter;
import org.usfirst.frc.team237.robot.commands.RotateTo;
import org.usfirst.frc.team237.robot.commands.StartFeeder;
import org.usfirst.frc.team237.robot.commands.StopFeeder;
import org.usfirst.frc.team237.robot.commands.StopRopeCommand;
import org.usfirst.frc.team237.robot.commands.StopShooter;
import org.usfirst.frc.team237.robot.commands.ToggleDriveOrientation;
import org.usfirst.frc.team237.robot.commands.ToggleGearLightCommand;
import org.usfirst.frc.team237.robot.commands.ZeroGyro;
import org.usfirst.frc.team237.robot.commands.autonomous.AlignToGear;
import org.usfirst.frc.team237.robot.subsystems.RopeSubsystem.Speed;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public static Joystick strafeJoystick = new Joystick(1);
	public static Joystick rotateJoystick = new Joystick(0);
	public static Joystick controls       = new Joystick(2);
	
	public static Button toggleFOD          = new JoystickButton(strafeJoystick, 1);
	public static Button toggleGearLight    = new JoystickButton(rotateJoystick, 1);
	public static Button leftPeg            = new JoystickButton(strafeJoystick, 4);
	public static Button middlePeg          = new JoystickButton(strafeJoystick, 3);
	public static Button rightPeg           = new JoystickButton(strafeJoystick, 5);
	public static Button zeroGyroButton     = new JoystickButton(strafeJoystick, 8);
	public static Button readyShooter       = new JoystickButton(controls, 1);
	public static Button outtakeButton      = new JoystickButton(controls, 2);
	public static Button intakeButton       = new JoystickButton(controls, 3);
	public static Button pullRopeFastButton = new JoystickButton(controls, 4);
	public static Button pullRopeSlowButton = new JoystickButton(controls, 5);
	public static Button autoHangGear       = new JoystickButton(controls, 6);
	public static Button shootButton        = new JoystickButton(controls, 7);
	
	public OI()
	{
		leftPeg            .whenPressed  (new RotateTo(60));
		middlePeg          .whenPressed  (new RotateTo(0));
		rightPeg           .whenPressed  (new RotateTo(-60));
		
		toggleFOD          .whenPressed  (new ToggleDriveOrientation());
		toggleGearLight    .whenPressed  (new ToggleGearLightCommand());
		zeroGyroButton     .whenPressed  (new ZeroGyro());
		
		autoHangGear       .whenPressed  (new AlignToGear());
//		autoHangGear       .whenReleased (new DriveForTimeAtSpeed(0, 0, 0, 0));
		
		readyShooter       .whenPressed  (new ReadyShooter());
		readyShooter       .whenReleased (new StopShooter());
		
		outtakeButton      .whenPressed  (new OuttakeCommand());
		outtakeButton      .whenReleased (new IntakeOffCommand());
		
		intakeButton       .whenPressed  (new IntakeCommand());
		intakeButton       .whenReleased (new IntakeOffCommand());
		
		pullRopeFastButton .whenPressed  (new PullRopeCommand(Speed.Fast));
		pullRopeFastButton .whenReleased (new StopRopeCommand());
		
		pullRopeSlowButton .whenPressed  (new PullRopeCommand(Speed.Slow));
		pullRopeSlowButton .whenReleased (new StopRopeCommand());
		
		shootButton        .whenPressed  (new StartFeeder());
		shootButton        .whenReleased (new StopFeeder());
	}
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
