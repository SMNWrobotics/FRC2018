package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutoUtil {
	
	static double leftTickPerRev = 30709; //number of pulses in an average rotation of the left wheel (4096 * 7.5 plus or minus a little bit (determined Jan 24))
	static double rightTickPerRev = 30736.2; //number of pulses in an average rotation of the right wheel (4096 * 7.5 plus or minus a little bit (determined Jan 24))
	
	static double wheelBaseWidth = 25.5; //(distance between left and right wheels)
	
	public static EncoderFollower[] getPath(Trajectory trajectory, double max_velocity) {
		//reset the gyro:
	    Robot.gyro.reset();
	    RobotMap.driveLeft.setSelectedSensorPosition(0, 0, 0);
		RobotMap.driveRight.setSelectedSensorPosition(0, 0, 0);
	    
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
	
	public static Trajectory generateTrajectory(Waypoint[] points, double max_velocity) {
		long oldTime;
		
		//trajectory for the robot to follow.  Made in one of two ways depending on the boolean generateNew
		Trajectory trajectory;
		
		Trajectory.Config config //make new configuration variable, with the following properties
	    = new Trajectory.Config( Trajectory.FitMethod.HERMITE_CUBIC
	       		               , Trajectory.Config.SAMPLES_HIGH
	       		               , 0.05 //time delta between points (seconds)
	       		               , max_velocity //max_velocity inches / second
	       		               , 25 //max_acceleration inches / second^2
	       		               , 80.0  //max_jerk inches / second^3
	       		               );
		
		System.out.println("Config created, Waypoints set, beginning generation of path!");
		oldTime = System.currentTimeMillis();
		
		trajectory = Pathfinder.generate(points, config); //generate segments for the trajectory based off of the waypoints and the config
		
		System.out.println("Finished generating the path! Time taken: " + (System.currentTimeMillis()-oldTime));
		
		return trajectory;
	}
	
	public static void printTrajectory(Trajectory trajectory, boolean onlyCsv) {
		//print out each segment's data (for future analysis using excel)
	    for (int i =0; i < trajectory.length(); i++) {
	    	Trajectory.Segment seg = trajectory.get(i);
	    	
	    	System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y, seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
	    }
	    if (!onlyCsv) {
	    	//print out each segment's data (for future use without the need to regenerate them)
	    	for (int i =0; i < trajectory.length(); i++) {
	    		Trajectory.Segment seg = trajectory.get(i);
	    		
	    		System.out.printf("new Segment(%f,%f,%f,%f,%f,%f,%f,%f),\n", seg.dt, seg.x, seg.y, seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
	    	}
	    }
	}
	
	//////////////////////////WARNING TO FUTURE USER: testing has shown that trusting the values from the encoders alone is more accurate than using the gyro's values
	public static double[] getMotorOutput(EncoderFollower left, EncoderFollower right) {
		int lpos = (int) Math.round(RobotMap.driveLeft.getSelectedSensorPosition(0));
		int rpos = (int) Math.round(RobotMap.driveRight.getSelectedSensorPosition(0));
		double l = left.calculate(lpos);
	    double r = right.calculate(rpos);
	    
	    double turn = 0.0;
	    double gyro_heading = -Robot.gyro.getAngleZ()/4; // Assuming the gyro is giving a value in degrees
	    if (Robot.gyroValid && Robot.useGyroForAuto) {
		    double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
		    
		    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		    turn = 0.8 * (-1.0/80.0) * angleDifference;
		    
		    SmartDashboard.putNumber("Gyro Angle", gyro_heading);
		    SmartDashboard.putNumber("Desired Heading", desired_heading);
	    }
	    
	    System.out.println("Lencoder: " + lpos + " Rencoder: " + rpos + " Left output: " + l + " Right output: " + r);// + " desired heading: " + desired_heading);
	    
	    double[] end = {l+turn, r-turn};
	    return end;
	}
	
}
