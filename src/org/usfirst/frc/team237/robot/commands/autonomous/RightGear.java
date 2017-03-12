package org.usfirst.frc.team237.robot.commands.autonomous;

import org.usfirst.frc.team237.robot.commands.DriveForCountsAtSpeed;
import org.usfirst.frc.team237.robot.commands.DriveForTimeAtSpeed;
import org.usfirst.frc.team237.robot.commands.RotateTo;
import org.usfirst.frc.team237.robot.commands.ToggleGearLightCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightGear extends CommandGroup {

    public RightGear() {
    	addSequential(new DriveForCountsAtSpeed (50000, 0.65, 0, 0));
		addSequential(new DriveForTimeAtSpeed(0.5, 0.0, 0, 0));
    	addSequential(new RotateTo(-67));
    	addSequential(new AlignToGear(1.75, 2.25));
    	addSequential(new DriveForTimeAtSpeed(0.5, 0.0, 0, 0));
    	addSequential(new DriveForCountsAtSpeed(15000, 0.2, 0, 0));
    	addSequential(new DriveForTimeAtSpeed(0.01, 0.2, -90, 0));
    	addSequential(new DriveForTimeAtSpeed(0.01, 0.2, 90, 0));
    	addSequential(new DriveForTimeAtSpeed(0.01, 0.2, -90, 0));
    	addSequential(new DriveForTimeAtSpeed(0.01, 0.2, 90, 0));
    	//addSequential(new DriveForCountsAtSpeed(1.2, 0.9, -2, 0));
    	//addSequential(new DriveForTimeAtSpeed(1.85, 0.42, 0, 0));
    	//addSequential(new DriveForCountsAtSpeed(0.25, 0, 0, 0));
    	//addSequential(new RotateTo(-60));
    	//addSequential(new DriveForCountsAtSpeed(0.1, 0.25, 0, 0));
    	//addSequential(new AlignToGear(1.9, 2));
    	//addSequential(new DriveForTimeAtSpeed(1, 0.1, 0, 0));
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
