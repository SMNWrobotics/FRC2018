package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class TogglePushPiston extends Command {

	protected void initialize() {
		if (RobotMap.pneumaticsPunch.get() == DoubleSolenoid.Value.kForward) {
			RobotMap.pneumaticsPunch.set(DoubleSolenoid.Value.kReverse);
			System.out.println("Setting Push Piston Solenoid to Reverse");
		} else {
			RobotMap.pneumaticsPunch.set(DoubleSolenoid.Value.kForward);
			System.out.println("Setting Push Piston Solenoid to Forward");
		}
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}

}
