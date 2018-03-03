package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorSwitchHeight extends Command {
	
	public ElevatorSwitchHeight() {
		requires(Robot.elevator);
	}
	
	@Override
	protected void execute() {
		RobotMap.elevatorRight.set(ControlMode.Position, 9250 ); //9250 is (tested) encoder ticks to get to 25 inches above the ground
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}
	
	@Override
	protected void end() {
		RobotMap.elevatorRight.set(ControlMode.PercentOutput, 0.0);
	}
	
	@Override
	protected void interrupted() {
		RobotMap.elevatorRight.set(ControlMode.PercentOutput, 0.0);
	}
	
}
