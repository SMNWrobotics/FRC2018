package org.usfirst.frc1982.the2018Bot.commands;

//NEGATIVE ANGLES TURN TO THE LEFT!!!

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnTo extends PIDCommand {
	
	private boolean done = false;
	private int ctr;
	private double maxSpeed = .8;
	private double deadZone = 0.0;//0.17;
	private double target;
	
	private volatile float out = (float) 0.0;
	
	public TurnTo( double degrees) {//, double maxSpeed) {
//		super(0.65/90.0, 0.0, 0.0); //finished auto in 7 seconds with only P value (maxSpeed = .4)
		super(1.25/90.0, 0.0002, 0.05);
		requires(Robot.drive);
		setInputRange(-95,95);
		setSetpoint(0);
		target = degrees;
	}
	
	protected void initialize() {
		Robot.gyro.reset();
		done = false;
		ctr = 0;
		out = (float) 0.0;
//		setSetpointRelative(degrees);
//		System.out.println("Initialize called");
	}
	
	protected void execute() {
		setSetpoint(target);
//		RobotMap.driveTrain.arcadeDrive(0.0, out);
//		RobotMap.driveleftSlave.set(RobotMap.driveLeft.getMotorOutputPercent());
//    	RobotMap.driverightSlave.set(RobotMap.driveRight.getMotorOutputPercent());
		arcadeDrive(0.0,out);
//		System.out.println("Execute called.  output: " + out);
	}
	
	@Override
	protected double returnPIDInput() {
		double currentAng = Robot.gyro.getAngleZ()/4;
		SmartDashboard.putNumber("Gyro Angle", currentAng);
		
		double diff = Math.abs(target - currentAng);
		if (diff < 5.0) { ctr++; }
		else { ctr = 0; }

//		System.out.println("ReturnPIDInput called");
		
		return currentAng;
	}

	@Override
	protected void usePIDOutput(double output) {
		if (output < deadZone && output > 0) output = deadZone;
		if (output > -deadZone && output < 0) output = -deadZone;
		if (output > maxSpeed) output = maxSpeed;
		if (output < -maxSpeed) output = -maxSpeed;
		
		SmartDashboard.putNumber("PID Output", output);
		
		out = (float) output;
		System.out.println("TurnTo PID output: " + out);
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
