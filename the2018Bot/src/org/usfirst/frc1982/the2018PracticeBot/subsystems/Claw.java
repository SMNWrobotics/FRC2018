package org.usfirst.frc1982.the2018Bot.subsystems;

import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Claw extends Subsystem {
	
	private DoubleSolenoid claw = RobotMap.pneumaticsClaw;
	
	@Override
	protected void initDefaultCommand() {
		
	}

}
