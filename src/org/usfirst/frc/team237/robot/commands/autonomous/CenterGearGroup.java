package org.usfirst.frc.team237.robot.commands.autonomous;

import org.usfirst.frc.team237.robot.commands.DriveForCountsAtSpeed;
import org.usfirst.frc.team237.robot.commands.DriveForTimeAtSpeed;
import org.usfirst.frc.team237.robot.commands.RotateTo;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CenterGearGroup extends CommandGroup {

    public CenterGearGroup() {
    	addSequential(new DriveForCountsAtSpeed(35000, 0.65, 0, 0));
    	addSequential(new DriveForTimeAtSpeed(0.5, 0.0, 0, 0));
    	addSequential(new AlignToGear(1.75, 2.25));
    	addSequential(new DriveForTimeAtSpeed(0.5, 0.0, 0, 0));
    	addSequential(new DriveForCountsAtSpeed(10000, 0.2, 0, 0));
    	addSequential(new DriveForTimeAtSpeed(0.01, 0.2, 90, 0));
    	addSequential(new DriveForTimeAtSpeed(0.01, 0.2, -90, 0));
    	addSequential(new DriveForTimeAtSpeed(0.01, 0.2, 90, 0));
    	addSequential(new DriveForTimeAtSpeed(0.01, 0.2, -90, 0));
    	//addSequential(new DriveForCountsAtSpeed(2.125, 0.25, 0, 0));
    	//addSequential(new AlignToGear(1.9, 2));
    	//addSequential(new DriveForCountsAtSpeed(0.5, 0.25, 0, 0));
    	//addSequential(new DriveForCountsAtSpeed(2, 0, 0, 0));
    	//addSequential(new DriveForCountsAtSpeed(1, 0.4, -180, 0));
    	//addSequential(new RotateTo(-90));
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
