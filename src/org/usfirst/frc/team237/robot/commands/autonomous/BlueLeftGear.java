package org.usfirst.frc.team237.robot.commands.autonomous;

import org.usfirst.frc.team237.robot.commands.DriveForTimeAtSpeed;
import org.usfirst.frc.team237.robot.commands.ReadyShooter;
import org.usfirst.frc.team237.robot.commands.RotateTo;
import org.usfirst.frc.team237.robot.commands.StartFeeder;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class BlueLeftGear extends CommandGroup {

    public BlueLeftGear() {
		addSequential(new DriveForTimeAtSpeed(1.75, 0.42, 7, 0));
    	addSequential(new RotateTo(55));
    	addSequential(new DriveForTimeAtSpeed(0.25, 0.3,0,0));
//    	addSequential(new AlignToGear(1.9, 2));
//    	addSequential(new DriveForTimeAtSpeed(1, 0.1, 0, 0));
//    	addSequential(new DriveForTimeAtSpeed(2, 0, 0, 0));
//    	addSequential(new DriveForTimeAtSpeed(2, 0.4, -180, 0));
//    	addSequential(new RotateTo(-120));
//    	addSequential(new DriveForTimeAtSpeed(.4, 0.4, -90, 0));
//    	addSequential(new ReadyShooter());
//    	addSequential(new StartFeeder());
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
