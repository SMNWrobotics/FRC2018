package org.usfirst.frc1982.the2018Bot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoSMidGSwLeft extends CommandGroup {
	public _AutoSMidGSwLeft() {
		double a = 24;
		double ang = 50.7;
		double b = 83.7;
		double c = 24;
		
		addSequential(new MoveToPID(a));
		addSequential(new TurnTo(-ang));
		addSequential(new MoveToPID(b));
		addSequential(new TurnTo(ang));
		addSequential(new MoveToPID(c));
	}
}
