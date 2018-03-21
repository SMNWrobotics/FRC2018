package org.usfirst.frc1982.the2018PracticeBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoSLeftGSwLeft extends CommandGroup {
	public _AutoSLeftGSwLeft() {
		double d1 = 136.5;
		double d2 = 36;
		addSequential(new MoveToPID(d1));
		addSequential(new TurnTo(90));
		addSequential(new MoveToPID(d2));
	}
}
