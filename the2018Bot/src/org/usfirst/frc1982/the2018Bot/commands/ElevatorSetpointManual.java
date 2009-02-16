package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorSetpointManual extends Command {
	
	public ElevatorSetpointManual() {
		requires(Robot.elevator);
	}
	
	@Override
	protected void initialize() {
		System.out.println("Setting position!!");
	}
	
	@Override
	protected void execute() {
		double value =  (((-Robot.oi.driver.getThrottle()+1)/2)*31000+5);
//		if (Robot.oi.driver.getThrottle() < 27000) {
			RobotMap.elevatorRight.set(ControlMode.Position, value);
//		} else if (Robot.oi.driver.getThrottle() < 27000){
//			RobotMap.elevatorRight.set(ControlMode.PercentOutput, .5);
//		}
//		System.out.println(RobotMap.elevatorRight.getSelectedSensorPosition(0));
		System.out.println("Temp pos: " + value + ", Encoder: " + RobotMap.elevatorRight.getSelectedSensorPosition(0));
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
