package org.usfirst.frc1982.the2018PracticeBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoSMidGSwLeft extends CommandGroup {
	
	public _AutoSMidGSwLeft() {
		addParallel(new AutoSMidGSwLeft());
		addParallel(new ToggleHinge(), 1);
		addSequential(new ToggleHinge(), 1);
		addParallel(new ElevatorSwitchHeight());
		addSequential(new ClawToggle(), 2);
		addSequential(new ClawToggle(), 2);
	}
	
}
