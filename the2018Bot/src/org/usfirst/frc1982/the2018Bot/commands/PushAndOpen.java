package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class PushAndOpen extends Command {

	long delay = 1000;
	long oldTime;
	
	protected void initialize() {
		System.out.println("(PushAndOpen) setting claw to forward and punch to forward");
		RobotMap.pneumaticsHinge.set(DoubleSolenoid.Value.kForward);
		RobotMap.pneumaticsPunch.set(DoubleSolenoid.Value.kForward);
		oldTime = System.currentTimeMillis();
	}
	
	@Override
	protected boolean isFinished() {
		if (System.currentTimeMillis() >= delay+oldTime) {
			RobotMap.pneumaticsPunch.set(DoubleSolenoid.Value.kReverse);
			return true;
		} else {
			return false;
		}
	}

}
