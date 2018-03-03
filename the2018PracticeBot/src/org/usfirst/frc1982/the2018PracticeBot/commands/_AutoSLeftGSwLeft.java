package org.usfirst.frc1982.the2018PracticeBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoSLeftGSwLeft extends CommandGroup {
	
	public _AutoSLeftGSwLeft() {
		addParallel(new AutoSLeftGSwLeft(), 19);
		addParallel(new ToggleHinge(), 1);
		addSequential(new ToggleHinge(), 1);
		addParallel(new ElevatorSwitchHeight());
		addSequential(new ClawToggle(), 2);
		addSequential(new ClawToggle(), 2);
	}
}
