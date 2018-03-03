package org.usfirst.frc1982.the2018Bot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoSMidGSwRight extends CommandGroup {
	
	public _AutoSMidGSwRight() {
		addParallel(new AutoSMidGSwRight());
		addParallel(new ToggleHinge(), 1);
		addSequential(new ToggleHinge(), 1);
		addParallel(new ElevatorSwitchHeight());
		addSequential(new ClawToggle(), 2);
		addSequential(new ClawToggle(), 2);
	}
}
