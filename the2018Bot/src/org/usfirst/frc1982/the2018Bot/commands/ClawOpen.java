package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class ClawOpen extends Command {
	
	long delay; //delay in milliseconds
	long oldTime;
	
	public ClawOpen(long MillisecondsDelay) {
		requires(Robot.claw);
		delay = MillisecondsDelay;
	}
	
	@Override
	protected void initialize() {
		oldTime = System.currentTimeMillis();
		if (delay == 0) {
			RobotMap.pneumaticsHinge.set(DoubleSolenoid.Value.kForward);
		}
	}
	
	@Override
	protected void execute() {
		if (System.currentTimeMillis()-oldTime >= delay) {
			RobotMap.pneumaticsHinge.set(DoubleSolenoid.Value.kForward);
		}
	}
	
	@Override
	protected boolean isFinished() {
		if (delay == 0 || System.currentTimeMillis()-oldTime >= delay+1000) {
			return true;
		} else {
			return false;
		}
	}
	
	@Override
	protected void end() {
		
	}
	
	@Override
	protected void interrupted() {
		
	}
}
