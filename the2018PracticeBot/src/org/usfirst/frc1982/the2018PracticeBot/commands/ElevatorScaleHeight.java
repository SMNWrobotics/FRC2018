package org.usfirst.frc1982.the2018PracticeBot.commands;

import org.usfirst.frc1982.the2018PracticeBot.Robot;
import org.usfirst.frc1982.the2018PracticeBot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorScaleHeight extends Command {

	public ElevatorScaleHeight() {
		requires(Robot.elevator);
	}
	
	@Override
	protected void execute() {
		RobotMap.elevatorRight.set(ControlMode.Position, 24650 ); //24650 is (tested) encoder ticks to get to 66 inches above the ground
		
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
