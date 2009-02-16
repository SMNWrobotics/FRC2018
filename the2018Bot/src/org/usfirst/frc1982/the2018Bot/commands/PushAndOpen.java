package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class PushAndOpen extends Command {

	long delay = 1000;
	long oldTime;
	
	public PushAndOpen() {
		super();
		requires(Robot.claw);
	}
	
	protected void initialize() {
		System.out.println("(PushAndOpen) setting claw to forward and punch to forward");
		if (RobotMap.pneumaticsClaw.get() == DoubleSolenoid.Value.kReverse) {
			RobotMap.pneumaticsClaw.set(DoubleSolenoid.Value.kForward);
		} else {
			RobotMap.pneumaticsClaw.set(DoubleSolenoid.Value.kReverse);
		}
		RobotMap.pneumaticsPunch.set(true);
		oldTime = System.currentTimeMillis();
	}
	
	@Override
	protected boolean isFinished() {
		if (System.currentTimeMillis() >= delay+oldTime) {
			RobotMap.pneumaticsPunch.set(false);
			return true;
		} else {
			return false;
		}
	}

}
