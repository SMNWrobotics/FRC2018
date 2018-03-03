package org.usfirst.frc1982.the2018PracticeBot.commands;

import org.usfirst.frc1982.the2018PracticeBot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class TogglePushPiston extends Command {

	protected void initialize() {
		if (RobotMap.pneumaticsPunch.get() == DoubleSolenoid.Value.kForward) {
			RobotMap.pneumaticsPunch.set(DoubleSolenoid.Value.kReverse);
		} else {
			RobotMap.pneumaticsPunch.set(DoubleSolenoid.Value.kForward);
		}
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}

}
