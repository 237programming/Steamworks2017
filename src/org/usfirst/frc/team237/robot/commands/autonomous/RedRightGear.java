package org.usfirst.frc.team237.robot.commands.autonomous;

import org.usfirst.frc.team237.robot.commands.DriveForTimeAtSpeed;
import org.usfirst.frc.team237.robot.commands.RotateTo;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RedRightGear extends CommandGroup {

    public RedRightGear() {
		addSequential(new DriveForTimeAtSpeed(2, 0.42, 0, 0));
    	addSequential(new RotateTo(-60));
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
