package org.usfirst.frc1982.the2018PracticeBot.commands;

import org.usfirst.frc1982.the2018PracticeBot.Robot;
import org.usfirst.frc1982.the2018PracticeBot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class MoveTo extends Command {
	
	private double distanceTravelled = 0.0;
	private int direction;
	private double target;
	
	
	public MoveTo(double distance) {
    	requires(Robot.drive);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    	this.target = distance;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	distanceTravelled = 0.0;
    	RobotMap.driveRight.setSelectedSensorPosition(0, 0, 0);
    	if (target > 0) {
    		direction = 1;
    	} else {
    		direction = -1;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	distanceTravelled = RobotMap.driveRight.getSelectedSensorPosition(0)*Robot.drive.inchesPerPulse;
//    	RobotMap.driveTrain.arcadeDrive(0.6*direction, 0.0);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    	if (Math.abs(distanceTravelled) >= Math.abs(target)) {
    		System.out.println("Auto command MoveTo should be finishing up");
    		return true;
    	}
    	return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
