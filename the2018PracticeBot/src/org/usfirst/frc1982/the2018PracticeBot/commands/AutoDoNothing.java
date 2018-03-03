package org.usfirst.frc1982.the2018PracticeBot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class AutoDoNothing extends Command {

	@Override
	protected void initialize() {
		System.out.println("Doing nothing!");
	}
	
	
	@Override
	protected boolean isFinished() {
		return false;
	}

}
