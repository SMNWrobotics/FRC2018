package org.usfirst.frc1982.the2018PracticeBot.commands;

import org.usfirst.frc1982.the2018PracticeBot.Robot;
import org.usfirst.frc1982.the2018PracticeBot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class driveEnable extends Command {

    public driveEnable() {
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	RobotMap.driveLeft.setSelectedSensorPosition(0, 0, 0);
    	RobotMap.driveRight.setSelectedSensorPosition(0, 0, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	SmartDashboard.putNumber("Gyro Angle", Robot.gyro.getAngleZ()/4);
    	
//    	RobotMap.driveTrain.arcadeDrive(-Robot.oi.xbox.getY(), Robot.oi.xbox.getThrottle());
//    	SmartDashboard.putNumber("Turning value:", Robot.oi.xbox.getThrottle());
    	
    	SmartDashboard.putNumber("LeftEncoder", RobotMap.driveLeft.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("RightEncoder", RobotMap.driveRight.getSelectedSensorPosition(0));
//    	RobotMap.driveLeft.set(-Robot.oi.xbox.getY());
//    	RobotMap.driveRight.set(-Robot.oi.xbox.getY());
//    	RobotMap.driveleftSlave.set(-Robot.oi.xbox.getY());
//    	RobotMap.driverightSlave.set(-Robot.oi.xbox.getY());
    	
    	arcadeDrive(-Robot.oi.driver.getY(), Robot.oi.driver.getTwist());
    	
    }
    
    private double limit(double input) {
    	if (input > 1.0) {
    		input = 1.0;
    	} else if (input < -1.0) {
    		input = -1.0;
    	}
    	return input;
    }
    
    private void arcadeDrive(double xSpeed, double zRotation) {
        xSpeed = limit(xSpeed);
//        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = limit(zRotation);
//        zRotation = applyDeadband(zRotation, m_deadband);

//        // Square the inputs (while preserving the sign) to increase fine control
//        // while permitting full power.
//        if (squaredInputs) {
//          xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
//          zRotation = Math.copySign(zRotation * zRotation, zRotation);
//        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed > 0.0) {
          // First quadrant, else second quadrant
          if (zRotation >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
          } else {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
          }
        } else if (xSpeed < 0.0){
          // Third quadrant, else fourth quadrant
          if (zRotation >= 0.0) {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
          } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
          }
        }else {
        	 leftMotorOutput = zRotation;
             rightMotorOutput = -zRotation;
        }

        RobotMap.driveLeft.set(limit(leftMotorOutput));
        RobotMap.driveleftSlave.set(limit(leftMotorOutput));
        
        RobotMap.driveRight.set(limit(rightMotorOutput));
        RobotMap.driverightSlave.set(limit(rightMotorOutput));
      }
    

    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	System.out.println("DriveEnable end() was called");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	
    }
}
