package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.RobotMap;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutoUtil {
	
	static double wheelBaseWidth = 25.5; //(distance between left and right wheels)
	
	public static EncoderFollower[] getPath(Trajectory trajectory, float max_velocity) {
		
		TankModifier modifier = new TankModifier(trajectory).modify(wheelBaseWidth);
		
		//set up tankDrive settings (so we can pull out the correct left and right wheel values)
		//make new encoderFollowers (gives the pathfinder a way to figure out where the robot is)
		EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
		EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());
		  
			    
		// Wheel Diameter is the diameter of your wheels (or pulley for a track system) in inches
		double wheel_diameter = 6;
		int initial_position = 0; //we are going to reset both encoders, so this value should be 0
		//reset both encoders
		RobotMap.driveLeft.setSelectedSensorPosition(0, 0, 0);
		RobotMap.driveRight.setSelectedSensorPosition(0, 0, 0);
		
		double leftTickPerRev = 30759; //number of pulses in an average rotation of the left wheel (4096 * 7.5 plus or minus a little bit (determined Jan 24))
		double rightTickPerRev = 30733.2; //number of pulses in an average rotation of the right wheel (4096 * 7.5 plus or minus a little bit (determined Jan 24))
			    
		//configuring the encoders based off of the wheel diameter, ticksperrev, and initial positions
		left.configureEncoder(initial_position, (int) leftTickPerRev, wheel_diameter);
		right.configureEncoder(initial_position, (int) rightTickPerRev, wheel_diameter);
			    
		// The first argument is the proportional gain. Usually this will be quite high
		// The second argument is the integral gain. This is unused for motion profiling
		// The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
		// The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
		//        trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
		// The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
		//configure the PID settings for controlling feedback and deviations from the setpoints
		left.configurePIDVA(.05, 0.0, 0.0, 1 / max_velocity, 0);
		right.configurePIDVA(.05, 0.0, 0.0, 1 / max_velocity, 0);
		
		EncoderFollower[] end = {left, right};
		return end;
		
	}
	
}
