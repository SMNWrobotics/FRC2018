package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class ToggleHinge extends Command {
	
	protected void initialize() {
		if (RobotMap.pneumaticsHinge.get() == DoubleSolenoid.Value.kForward) {
			RobotMap.pneumaticsHinge.set(DoubleSolenoid.Value.kReverse);
			System.out.println("setting Hinge Solenoid to Reverse");
		} else {
			RobotMap.pneumaticsHinge.set(DoubleSolenoid.Value.kForward);
			System.out.println("Setting Hinge Solenoid to Forward");
		}
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}

}
