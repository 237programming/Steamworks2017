package org.usfirst.frc.team237.robot;

import org.usfirst.frc.team237.robot.commands.ReadyShooter;
import org.usfirst.frc.team237.robot.commands.StopShooter;
import org.usfirst.frc.team237.robot.commands.ToggleDriveOrientation;
import org.usfirst.frc.team237.robot.commands.ZeroGyro;

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
	public static Joystick controls = new Joystick(2);
	
	public static Button toggleFOD = new JoystickButton(strafeJoystick, 1);
	public static Button zeroGyro = new JoystickButton(rotateJoystick, 1);
	public static Button readyShooter = new JoystickButton(controls, 1);
	
	public OI()
	{
		toggleFOD.whenPressed(new ToggleDriveOrientation());
		zeroGyro.whenPressed(new ZeroGyro());
		readyShooter.whenPressed(new ReadyShooter());
		readyShooter.whenReleased(new StopShooter());
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
