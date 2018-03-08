package org.usfirst.frc1982.the2018PracticeBot.commands;

import org.frc1982.common.AutoChooser;
import org.frc1982.common.Goal;
import org.frc1982.common.Position;
import org.frc1982.common.Side;
import org.usfirst.frc1982.the2018PracticeBot.Robot;
import org.usfirst.frc1982.the2018PracticeBot.RobotMap;
import org.usfirst.frc1982.the2018PracticeBot.commands.AutoUtil;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.Trajectory.Segment;

public class AutoUsingUtil extends Command {
	
	EncoderFollower left;
	EncoderFollower right;
	public static Segment[] path;
	
	@Override
	protected void initialize() {
		Trajectory trajectory;
		
		trajectory = new Trajectory(path /*AutoChooser.getAutoPath(Position.MID, Goal.SWITCH, Side.LEFT)*/);
		
		EncoderFollower[] result = AutoUtil.getPath(trajectory, Robot.max_velocity);
		
		left = result[0];
		right = result[1];
	}
	
	@Override
	protected void execute() {
		double[] outputs = AutoUtil.getMotorOutput(left, right);
		
		RobotMap.driveLeft.set(outputs[0]);
	    RobotMap.driveleftSlave.set(outputs[0]);
	    RobotMap.driveRight.set(outputs[1]);
    	RobotMap.driverightSlave.set(outputs[1]);
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

}
