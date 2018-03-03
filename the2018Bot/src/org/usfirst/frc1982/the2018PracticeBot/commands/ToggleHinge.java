package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class ToggleHinge extends Command {
	
	protected void initialize() {
		if (RobotMap.pneumaticsHinge.get() == DoubleSolenoid.Value.kForward) {
			RobotMap.pneumaticsHinge.set(DoubleSolenoid.Value.kReverse);
		} else {
			RobotMap.pneumaticsHinge.set(DoubleSolenoid.Value.kForward);
		}
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}

}
