package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDCommand;

public class MoveToPID extends PIDCommand {
	
	private boolean done = false;
	private int ctr;
	private double maxSpeed = .8;
	private double deadZone = 0.0;//0.17;
	private double target;
	
	private volatile float out = (float) 0.0;
	
	public MoveToPID(double distance) {
		super(0.07, 0.0, 0.06);
		setInputRange(-1000,1000);
		setSetpoint(0);
		target = distance;
	}
	
	protected void initialize() {
		System.out.println("MoveToPID started (initialized)");
		ctr = 0;
		done = false;
		RobotMap.driveLeft.setSelectedSensorPosition(0, 0, 0);
		RobotMap.driveRight.setSelectedSensorPosition(0, 0, 0);
	}
	
	protected void execute() {
		setSetpoint(target);
		arcadeDrive(out,0.0);
	}
	
	@Override
	protected double returnPIDInput() {
		double currentDist = (RobotMap.driveLeft.getSelectedSensorPosition(0)*Robot.drive.LeftInchesPerPulse + RobotMap.driveRight.getSelectedSensorPosition(0)*Robot.drive.RightInchesPerPulse) / 2.0;
		
		double diff = Math.abs(target - currentDist);
		if (diff < 5.0) { ctr++; }
		else { ctr = 0; }

		System.out.println("Distance: " + currentDist + " Target: " + getSetpoint() + " PIDout: " + out);
		
		return currentDist;
	}

	@Override
	protected void usePIDOutput(double output) {
		if (output < deadZone && output > 0) output = deadZone;
		if (output > -deadZone && output < 0) output = -deadZone;
		if (output > maxSpeed) output = maxSpeed;
		if (output < -maxSpeed) output = -maxSpeed;
		
		out = (float) output;
	}

	@Override
	protected boolean isFinished() {
		if (ctr > 7) {
			done = true;
			RobotMap.driveRight.setSelectedSensorPosition(0, 0, 0);
	    	RobotMap.driveLeft.setSelectedSensorPosition(0, 0, 0);
		} else {
			done = false;
		}
		return done;
	}

/////borrowed from driveEnable command:
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
//        RobotMap.driveleftSlave.set(limit(leftMotorOutput));
        
        RobotMap.driveRight.set(limit(rightMotorOutput));
//        RobotMap.driverightSlave.set(limit(rightMotorOutput));
      }
	
}
