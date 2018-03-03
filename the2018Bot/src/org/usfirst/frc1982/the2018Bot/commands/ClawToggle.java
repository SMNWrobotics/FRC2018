package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class ClawToggle extends Command {
	
	public ClawToggle() {
		requires(Robot.claw);
	}
	
	@Override
	protected void initialize() {
		if (RobotMap.pneumaticsClaw.get() == DoubleSolenoid.Value.kForward) {
			RobotMap.pneumaticsClaw.set(DoubleSolenoid.Value.kReverse);
			System.out.println("Setting Claw Solennoid to Reverse");
		} else {
			RobotMap.pneumaticsClaw.set(DoubleSolenoid.Value.kForward);
			System.out.println("Setting Claw Solenoid to Forward");
		}
	}
	
	@Override
	protected void execute() {
		
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}
	
	@Override
	protected void end() {
		
	}
	
	@Override
	protected void interrupted() {
		
	}

}
