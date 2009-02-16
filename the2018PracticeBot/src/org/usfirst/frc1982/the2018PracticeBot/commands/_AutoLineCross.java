package org.usfirst.frc1982.the2018PracticeBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoLineCross extends CommandGroup {
	public _AutoLineCross() {
		addSequential(new MoveToPID(140));
	}
}
