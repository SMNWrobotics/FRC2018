package org.usfirst.frc1982.the2018Bot.commands;

import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutoWaypointTest extends Command {
	
	private boolean generateNew = true;
	
	EncoderFollower left;
	EncoderFollower right;
	
	@Override
	protected void initialize() {
		//timing variable for the println statements
		long oldTime;
		
		//max_velocity inches / second
		double max_velocity = 20; 
		
		//trajectory for the robot to follow.  Made in one of two ways depending on the boolean generateNew
		Trajectory trajectory;
		
		//if we want to generate a new trajectory (for instance to record the points, etc.)
		if (generateNew) {
			Trajectory.Config config //make new configuration variable, with the following properties
	        = new Trajectory.Config( Trajectory.FitMethod.HERMITE_CUBIC
	        		               , Trajectory.Config.SAMPLES_HIGH
	        		               , 0.05 //time delta between points (seconds)
	        		               , max_velocity //max_velocity inches / second
	        		               , 10 //max_acceleration inches / second^2
	        		               , 50.0  //max_jerk inches / second^3
	        		               );
			//waypoint comments: x is forwards, y is sideways
			//negative angle is rotate counterclockwise, + is rotate clockwise
			Waypoint[] points = new Waypoint[] { //list of waypoints to go through
				new Waypoint(0.0, 0.0, 0.0),
				new Waypoint(35.0,0.0,0.0),
				new Waypoint(50.0,-5.0,Pathfinder.d2r(-45)),
				new Waypoint(75.75,-10.0,Pathfinder.d2r(-90)),
				new Waypoint(75.75,-15.0,Pathfinder.d2r(-90))
				
//				new Waypoint(0,-4,0),
//				new Waypoint(0.2,-2,0),
//				new Waypoint(0,0,0)
				
//	    		new Waypoint(0d, 2d, 0d),
//	    		new Waypoint(2d,2d,0d),
//	    		new Waypoint(2d, 4d, 0d)
//	    		new Waypoint(-4, -1, Pathfinder.d2r(-45)), 
//	    		new Waypoint(-2, -2, 0)
			};
			
			System.out.println("Config created, Waypoints set, beginning generation of path!");
			oldTime = System.currentTimeMillis();
			
			trajectory = Pathfinder.generate(points, config); //generate segments for the trajectory based off of the waypoints and the config
			
			System.out.println("Finished generating the path! Time taken: " + (System.currentTimeMillis()-oldTime));
		
		} else { //if we don't want to make a new trajectory (import from attribute segs below)
			
			//make new trajectory based solely off of the imported segments declared above
			trajectory = new Trajectory(segs); 
			
		}
		// Wheelbase Width is the distance from the left wheel to the right wheel, in this case 25.5 (inches)
		double wheelbaseWidth = 25.5;
		
		//set up tankDrive settings (so we can pull out the correct left and right wheel values)
		TankModifier modifier = new TankModifier(trajectory).modify(wheelbaseWidth);
		
		
	    //make new encoderFollowers (gives the pathfinder a way to figure out where the robot is)
	    left = new EncoderFollower(modifier.getLeftTrajectory());
	    right = new EncoderFollower(modifier.getRightTrajectory());
	    
	    
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
	    
	    if (generateNew) { //if generating new points:
	    	//print out each segment's data (for future use without the need to regenerate them)
		    for (int i =0; i < trajectory.length(); i++) {
		    	Trajectory.Segment seg = trajectory.get(i);
		    	
		    	System.out.printf("new Segment(%f,%f,%f,%f,%f,%f,%f,%f),\n", seg.dt, seg.x, seg.y, seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		    }
	    }
	    //reset the gyro:
	    Robot.gyro.reset();
	}
	
	@Override
	protected void execute() {
		//7.44 is magic number determined by rotating wheel about 1 revolution
		int lpos = (int) Math.round(RobotMap.driveLeft.getSelectedSensorPosition(0));
		int rpos = (int) Math.round(RobotMap.driveRight.getSelectedSensorPosition(0));
		double l = left.calculate(lpos);
	    double r = right.calculate(rpos);
	    
	    double gyro_heading = -Robot.gyro.getAngleZ()/4; // Assuming the gyro is giving a value in degrees
	    double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
	    
	    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
	    double turn = 0.8 * (-1.0/80.0) * angleDifference;

	    SmartDashboard.putNumber("Gyro Angle", gyro_heading);
	    SmartDashboard.putNumber("Desired Heading", desired_heading);
	    System.out.println("Lencoder: " + (lpos/30759*6) + " Rencoder: " + rpos + " Left output: " + l + " Right output: " + r);// + " desired heading: " + desired_heading);
	    
	    RobotMap.driveLeft.set(l + turn);
	    RobotMap.driveleftSlave.set(l + turn);
	    RobotMap.driveRight.set(r - turn);
    	RobotMap.driverightSlave.set(r - turn);
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

	private Segment[] segs = {
			
	};
	
}
