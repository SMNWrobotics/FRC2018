package org.usfirst.frc1982.the2018PracticeBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoFarLine extends CommandGroup {
	public _AutoFarLine() {
		addSequential(new MoveToPID(210));
	}
}
