package org.usfirst.frc1982.the2018PracticeBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoTest extends CommandGroup {
	
	public _AutoTest() {
//		addSequential(new MoveTo(24));
		addSequential(new TurnTo( 90, .7));
	}
	
}
