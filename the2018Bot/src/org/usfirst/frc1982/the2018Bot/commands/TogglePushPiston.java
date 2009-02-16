package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class TogglePushPiston extends Command {

	protected void initialize() {
		if (RobotMap.pneumaticsPunch.get()) {
			RobotMap.pneumaticsPunch.set(false);
			System.out.println("Setting Push Piston Solenoid to false");
		} else {
			RobotMap.pneumaticsPunch.set(true);
			System.out.println("Setting Push Piston Solenoid to true");
		}
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}

}
