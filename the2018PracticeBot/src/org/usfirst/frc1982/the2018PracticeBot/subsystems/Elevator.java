package org.usfirst.frc1982.the2018PracticeBot.subsystems;

import org.usfirst.frc1982.the2018PracticeBot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {
	
	private final WPI_TalonSRX elevatorRight = RobotMap.driveRight;
	private final WPI_TalonSRX elevatorLeft = RobotMap.driveLeft;
	
	@Override
	protected void initDefaultCommand() {
		
	}
	
	private int ctr = 0;
	
	@Override
	public void periodic() {
//		System.out.println(RobotMap.elevatorRight.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("LiftEncoder", elevatorRight.getSelectedSensorPosition(0));
		ctr++;
		if (ctr % 50 == 0) {
//			System.out.println("Rev Right: " + (elevatorRight.getSensorCollection().isRevLimitSwitchClosed()) + ", For Right: " + elevatorRight.getSensorCollection().isFwdLimitSwitchClosed() + ", Rev Left: " + elevatorLeft.getSensorCollection().isRevLimitSwitchClosed() + ", Left For: " + elevatorLeft.getSensorCollection().isFwdLimitSwitchClosed() + ", ctr: " + (ctr));
		}
	}

}
