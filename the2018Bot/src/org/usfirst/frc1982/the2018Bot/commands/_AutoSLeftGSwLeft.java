package org.usfirst.frc1982.the2018Bot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class _AutoSLeftGSwLeft extends CommandGroup {
	
	public _AutoSLeftGSwLeft() {
		///////obsolete, follow _AutoSMidGSwLeft for current format!
		addParallel(new AutoSLeftGSwLeft(), 19);
		addParallel(new ToggleHinge(), 1);
		addParallel(new ElevatorSwitchHeight());
		addSequential(new ClawToggle(), 2);
		addSequential(new ClawToggle(), 2);
	}
	protected void initialize() {
		super.initialize();
		System.out.println("Autonomous Starting Middle going to left switch has begun");
	}
}
