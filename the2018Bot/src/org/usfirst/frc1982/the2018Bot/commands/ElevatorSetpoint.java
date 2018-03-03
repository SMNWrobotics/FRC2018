package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorSetpoint extends Command {
	
	public ElevatorSetpoint() {
		requires(Robot.elevator);
	}
	
	@Override
	protected void initialize() {
		System.out.println("Setting position!!");
		RobotMap.elevatorRight.set(ControlMode.Position, (((Robot.oi.opBox.getX()+1)/2)*27100) );
	}
	
	@Override
	protected void execute() {
		RobotMap.elevatorRight.set(ControlMode.Position, (((Robot.oi.opBox.getX()+1)/2)*27100) );
		System.out.println("Encoder: " + RobotMap.elevatorRight.getSelectedSensorPosition(0) + "SetPoint: " + ((Robot.oi.opBox.getX()+1)/2)*27100);
//		System.out.println("Temp pos: " + (((-Robot.oi.xbox.getZ()+1)/2)*20000+5) + "Encoder: " + RobotMap.elevatorRight.getSelectedSensorPosition(0) + " Talon Output: " + RobotMap.elevatorRight.get());
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
