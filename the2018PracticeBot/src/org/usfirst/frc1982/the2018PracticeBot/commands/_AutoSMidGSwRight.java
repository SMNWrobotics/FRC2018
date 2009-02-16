package org.usfirst.frc1982.the2018PracticeBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoSMidGSwRight extends CommandGroup {
	public _AutoSMidGSwRight() {
		double a = 24;
		double ang = 45;
		double b = 76.2;
		double c = 24;
		
		addSequential(new MoveToPID(a));
		addSequential(new TurnTo(ang));
		addSequential(new MoveToPID(b));
		addSequential(new TurnTo(-ang));
		addSequential(new MoveToPID(c));
	}
}
