package org.usfirst.frc1982.the2018PracticeBot.commands;

import org.usfirst.frc1982.the2018PracticeBot.Robot;
import org.usfirst.frc1982.the2018PracticeBot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorManual extends Command {
	
	public ElevatorManual() {
		requires(Robot.elevator);
	}
	
	@Override
	protected void execute() {
		RobotMap.elevatorRight.set(ControlMode.PercentOutput, -Robot.oi.driver.getThrottle());
		System.out.println("Encoder Value: " + RobotMap.elevatorRight.getSelectedSensorPosition(0));
//		RobotMap.elevatorRight.Is
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
