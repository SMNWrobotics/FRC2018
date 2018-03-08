package org.usfirst.frc1982.the2018PracticeBot.commands;

import org.frc1982.common.Goal;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory.Segment;

public class _AutoGeneral extends CommandGroup {
	public _AutoGeneral(Segment[] path, Goal g) {
		AutoUsingUtil.path = path;
		addParallel(new AutoUsingUtil());
		
		if (g == Goal.SWITCH) {
			addParallel(new ElevatorSwitchHeight()); //set elevator to switch height
			addParallel(new ClawOpen(2500)); //open claw after 2.5 seconds
		} else if (g == Goal.SCALE) {
			//UNTESTED:
			addParallel(new ElevatorScaleHeight()); //set elevator to scale height
			addParallel(new ClawOpen(7000)); //open claw after 7 seconds
		} //if the goal isn't one of these two, then these extra commands do not need to be run
	}
	
	
}
