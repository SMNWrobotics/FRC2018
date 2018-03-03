package org.usfirst.frc1982.the2018Bot.commands;



import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnTo extends PIDCommand {
	
	private boolean done = false;
	private int ctr;
	private double maxSpeed;
	private double deadZone = 0.4;
	private double setPoint;
	
	private volatile float out = (float) 0.0;
	
	public TurnTo( double degrees, double maxSpeed) {
		super(0.6/90.0, 0.0, 0.0);
		SmartDashboard.putData("PID Command", this);
		requires(Robot.drive);
		setSetpoint(0);
		setSetpointRelative(degrees);
		setPoint = degrees;
		this.maxSpeed = Math.abs(maxSpeed);
	}
	
	protected void initialize() {
		Robot.gyro.reset();
		done = false;
		ctr = 0;
		out = (float) 0.0;
	}
	
	protected void execute() {
//		RobotMap.driveTrain.arcadeDrive(0.0, out);
		RobotMap.driveleftSlave.set(RobotMap.driveLeft.getMotorOutputPercent());
    	RobotMap.driverightSlave.set(RobotMap.driveRight.getMotorOutputPercent());
	}
	
	@Override
	protected double returnPIDInput() {
		double currentAng = Robot.gyro.getAngleZ()/4;
		SmartDashboard.putNumber("Gyro Angle", currentAng);
		
		double diff = Math.abs(setPoint - currentAng);
		if (diff < 5.0) { ctr++; }
		else { ctr = 0; }
		
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
	}

	@Override
	protected boolean isFinished() {
		if (ctr > 7) {
			done = true;
		} else {
			done = false;
		}
		return done;
	}
	
}
