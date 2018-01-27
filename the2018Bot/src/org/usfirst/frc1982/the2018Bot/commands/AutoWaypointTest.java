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
	
	private boolean generateNew = false;
	private boolean generateOnly;
	
	EncoderFollower left;
	EncoderFollower right;
	
	@Override
	protected void initialize() {
		generateOnly = SmartDashboard.getBoolean("Only Generate Autonomous", true);
		//timing variable for the println statements
		long oldTime;
		
		//max_velocity inches / second
		double max_velocity = 100; 
		
		//trajectory for the robot to follow.  Made in one of two ways depending on the boolean generateNew
		Trajectory trajectory;
		
		//if we want to generate a new trajectory (for instance to record the points, etc.)
		if (generateNew) {
			Trajectory.Config config //make new configuration variable, with the following properties
	        = new Trajectory.Config( Trajectory.FitMethod.HERMITE_CUBIC
	        		               , Trajectory.Config.SAMPLES_HIGH
	        		               , 0.05 //time delta between points (seconds)
	        		               , max_velocity //max_velocity inches / second
	        		               , 15 //max_acceleration inches / second^2
	        		               , 50.0  //max_jerk inches / second^3
	        		               );
			//waypoint comments: x is forwards, y is sideways
			//negative angle is rotate counterclockwise, + is rotate clockwise
			//list of waypoints to go through
			Waypoint[] points = new Waypoint[] {
				new Waypoint(0.0,0.0,0.0),
				new Waypoint(273.5,0.0,0.0),
				new Waypoint(291,-5.38,Pathfinder.d2r(90))
				
					//AutoSMidGRight:
//				new Waypoint(0,0,0),
//				new Waypoint(105.5,-59.0,0.0)
				
					//AutoSMidGLeft:
//				new Waypoint(0,0,0),
//				new Waypoint(105.5,62.0,0.0)
				
					//AutoSleftGLeft
//				new Waypoint(0.0, 0.0, 0.0),
//				new Waypoint(50.0,0.0,0.0),
//				new Waypoint(75.0,0.0,0.0),
//				new Waypoint(100.0,0.0,0.0),
//				new Waypoint(151.5,-29.06,Pathfinder.d2r(90))
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
	    	if (generateOnly) {
		    	//print out each segment's data (for future analysis using excel)
			    for (int i =0; i < trajectory.length(); i++) {
			    	Trajectory.Segment seg = trajectory.get(i);
			    	
			    	System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y, seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
			    }
			    //print out each segment's data (for future use without the need to regenerate them)
			    for (int i =0; i < trajectory.length(); i++) {
			    	Trajectory.Segment seg = trajectory.get(i);
			    	
			    	System.out.printf("new Segment(%f,%f,%f,%f,%f,%f,%f,%f),\n", seg.dt, seg.x, seg.y, seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
			    }
	    	} else {
	    		//print out each segment's data (for future analysis using excel)
			    for (int i =0; i < trajectory.length(); i++) {
			    	Trajectory.Segment seg = trajectory.get(i);
			    	
			    	System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y, seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
			    }
	    	}
	    }
	    //reset the gyro:
	    Robot.gyro.reset();
	}
	
	@Override
	protected void execute() {
		if (!generateOnly) {
			//7.44 is magic number determined by rotating wheel about 1 revolution
			int lpos = (int) Math.round(RobotMap.driveLeft.getSelectedSensorPosition(0));
			int rpos = (int) Math.round(RobotMap.driveRight.getSelectedSensorPosition(0));
			double l = left.calculate(lpos);
		    double r = right.calculate(rpos);
		    
		    double turn = 0.0;
		    double gyro_heading = -Robot.gyro.getAngleZ()/4; // Assuming the gyro is giving a value in degrees
		    if (Robot.gyroValid) {
			    double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
			    
			    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
			    turn = 0.8 * (-1.0/80.0) * angleDifference;
			    
			    SmartDashboard.putNumber("Gyro Angle", gyro_heading);
			    SmartDashboard.putNumber("Desired Heading", desired_heading);
		    }
		    System.out.println("Lencoder: " + (lpos/30759*6) + " Rencoder: " + rpos + " Left output: " + l + " Right output: " + r);// + " desired heading: " + desired_heading);
		    
		    RobotMap.driveLeft.set(l + turn);
		    RobotMap.driveleftSlave.set(l + turn);
		    RobotMap.driveRight.set(r - turn);
	    	RobotMap.driverightSlave.set(r - turn);
		}
	}
	
	@Override
	protected boolean isFinished() {
		return generateOnly;
	}

	private Segment[] segs = {
			new Segment(0.050000,0.000376,0.000000,0.003111,0.124430,2.488608,49.772166,0.000000),
			new Segment(0.050000,0.012819,0.000000,0.015554,0.373291,4.977217,49.772166,0.000000),
			new Segment(0.050000,0.040816,0.000000,0.043551,0.746582,7.465825,49.772166,0.000000),
			new Segment(0.050000,0.090588,0.000000,0.093323,1.244304,9.954433,49.772166,0.000000),
			new Segment(0.050000,0.168357,0.000000,0.171092,1.866456,12.443042,49.772166,0.000000),
			new Segment(0.050000,0.280344,0.000000,0.283079,2.613039,14.931650,49.772166,0.000000),
			new Segment(0.050000,0.429661,0.000000,0.432396,3.359621,14.931650,0.000000,0.000000),
			new Segment(0.050000,0.616306,0.000000,0.619041,4.106204,14.931650,-0.000000,0.000000),
			new Segment(0.050000,0.840281,0.000000,0.843016,4.852786,14.931650,0.000000,0.000000),
			new Segment(0.050000,1.101585,0.000000,1.104320,5.599369,14.931650,-0.000000,0.000000),
			new Segment(0.050000,1.400218,0.000000,1.402953,6.345951,14.931650,-0.000000,0.000000),
			new Segment(0.050000,1.736180,0.000000,1.738915,7.092534,14.931650,0.000000,0.000000),
			new Segment(0.050000,2.109471,0.000000,2.112206,7.839116,14.931650,0.000000,0.000000),
			new Segment(0.050000,2.520092,0.000000,2.522827,8.585699,14.931650,-0.000000,0.000000),
			new Segment(0.050000,2.968041,0.000000,2.970776,9.332281,14.931650,0.000000,0.000000),
			new Segment(0.050000,3.453320,0.000000,3.456055,10.078864,14.931650,0.000000,0.000000),
			new Segment(0.050000,3.975928,0.000000,3.978663,10.825446,14.931650,0.000000,0.000000),
			new Segment(0.050000,4.535864,0.000000,4.538599,11.572029,14.931650,-0.000000,0.000000),
			new Segment(0.050000,5.133130,0.000000,5.135865,12.318611,14.931650,0.000000,0.000000),
			new Segment(0.050000,5.767726,0.000000,5.770461,13.065194,14.931650,-0.000000,0.000000),
			new Segment(0.050000,6.439650,0.000000,6.442385,13.811776,14.931650,0.000000,0.000000),
			new Segment(0.050000,7.148903,0.000000,7.151638,14.558359,14.931650,0.000000,0.000000),
			new Segment(0.050000,7.895486,0.000000,7.898221,15.304941,14.931650,-0.000000,0.000000),
			new Segment(0.050000,8.679397,0.000000,8.682132,16.051524,14.931650,0.000000,0.000000),
			new Segment(0.050000,9.500638,0.000000,9.503373,16.798106,14.931650,0.000000,0.000000),
			new Segment(0.050000,10.359208,0.000000,10.361943,17.544689,14.931650,-0.000000,0.000000),
			new Segment(0.050000,11.255107,0.000000,11.257842,18.291271,14.931650,0.000000,0.000000),
			new Segment(0.050000,12.188335,0.000000,12.191070,19.037854,14.931650,0.000000,0.000000),
			new Segment(0.050000,13.158892,0.000000,13.161627,19.784436,14.931650,0.000000,0.000000),
			new Segment(0.050000,14.166779,0.000000,14.169514,20.531019,14.931650,-0.000000,0.000000),
			new Segment(0.050000,15.211994,0.000000,15.214729,21.277601,14.931650,0.000000,0.000000),
			new Segment(0.050000,16.294539,0.000000,16.297274,22.024184,14.931650,0.000000,0.000000),
			new Segment(0.050000,17.414412,0.000000,17.417147,22.770766,14.931650,0.000000,0.000000),
			new Segment(0.050000,18.571615,0.000000,18.574350,23.517349,14.931650,0.000000,0.000000),
			new Segment(0.050000,19.766147,0.000000,19.768882,24.263931,14.931650,-0.000000,0.000000),
			new Segment(0.050000,20.998008,0.000000,21.000743,25.010514,14.931650,0.000000,0.000000),
			new Segment(0.050000,22.267199,0.000000,22.269934,25.757096,14.931650,0.000000,0.000000),
			new Segment(0.050000,23.573718,0.000000,23.576453,26.503679,14.931650,-0.000000,0.000000),
			new Segment(0.050000,24.917567,0.000000,24.920302,27.250261,14.931650,0.000000,0.000000),
			new Segment(0.050000,26.298744,0.000000,26.301479,27.996844,14.931650,-0.000000,0.000000),
			new Segment(0.050000,27.717251,0.000000,27.719986,28.743426,14.931650,-0.000000,0.000000),
			new Segment(0.050000,29.173087,0.000000,29.175822,29.490009,14.931650,0.000000,0.000000),
			new Segment(0.050000,30.666252,0.000000,30.668987,30.236591,14.931650,0.000000,0.000000),
			new Segment(0.050000,32.196746,0.000000,32.199481,30.983174,14.931650,0.000000,0.000000),
			new Segment(0.050000,33.764569,0.000000,33.767304,31.729756,14.931650,-0.000000,0.000000),
			new Segment(0.050000,35.369721,0.000000,35.372456,32.476339,14.931650,0.000000,0.000000),
			new Segment(0.050000,37.012203,0.000000,37.014938,33.222921,14.931650,0.000000,0.000000),
			new Segment(0.050000,38.692014,0.000000,38.694749,33.969504,14.931650,-0.000000,0.000000),
			new Segment(0.050000,40.409153,0.000000,40.411888,34.716086,14.931650,0.000000,0.000000),
			new Segment(0.050000,42.163622,0.000000,42.166357,35.462669,14.931650,0.000000,0.000000),
			new Segment(0.050000,43.955420,0.000000,43.958155,36.209251,14.931650,-0.000000,0.000000),
			new Segment(0.050000,45.784547,0.000000,45.787282,36.955834,14.931650,0.000000,0.000000),
			new Segment(0.050000,47.651004,0.000000,47.653739,37.702416,14.931650,-0.000000,0.000000),
			new Segment(0.050000,49.554789,0.000000,49.557524,38.448998,14.931650,0.000000,0.000000),
			new Segment(0.050000,51.495903,0.000000,51.498638,39.195581,14.931650,-0.000000,0.000000),
			new Segment(0.050000,53.474347,0.000000,53.477082,39.942163,14.931650,0.000000,0.000000),
			new Segment(0.050000,55.490120,0.000000,55.492855,40.688746,14.931650,0.000000,0.000000),
			new Segment(0.050000,57.543222,0.000000,57.545957,41.435328,14.931650,-0.000000,0.000000),
			new Segment(0.050000,59.633653,0.000000,59.636388,42.181911,14.931650,0.000000,0.000000),
			new Segment(0.050000,61.761413,0.000000,61.764148,42.928493,14.931650,-0.000000,0.000000),
			new Segment(0.050000,63.926502,0.000000,63.929237,43.675076,14.931650,0.000000,0.000000),
			new Segment(0.050000,66.128920,0.000000,66.131655,44.421658,14.931650,0.000000,0.000000),
			new Segment(0.050000,68.368668,0.000000,68.371403,45.168241,14.931650,-0.000000,0.000000),
			new Segment(0.050000,70.645744,0.000000,70.648479,45.914823,14.931650,0.000000,0.000000),
			new Segment(0.050000,72.960150,0.000000,72.962885,46.661406,14.931650,0.000000,0.000000),
			new Segment(0.050000,75.311885,0.000000,75.314620,47.407988,14.931650,0.000000,0.000000),
			new Segment(0.050000,77.700949,0.000000,77.703684,48.154571,14.931650,-0.000000,0.000000),
			new Segment(0.050000,80.127342,0.000000,80.130077,48.901153,14.931650,0.000000,0.000000),
			new Segment(0.050000,82.591064,0.000000,82.593799,49.647736,14.931650,0.000000,0.000000),
			new Segment(0.050000,85.092116,0.000000,85.094851,50.394318,14.931650,-0.000000,0.000000),
			new Segment(0.050000,87.630496,0.000000,87.633231,51.140901,14.931650,0.000000,0.000000),
			new Segment(0.050000,90.206206,0.000000,90.208941,51.887483,14.931650,-0.000000,0.000000),
			new Segment(0.050000,92.819244,0.000000,92.821979,52.634066,14.931650,0.000000,0.000000),
			new Segment(0.050000,95.469612,0.000000,95.472347,53.380648,14.931650,0.000000,0.000000),
			new Segment(0.050000,98.157309,0.000000,98.160044,54.127231,14.931650,-0.000000,0.000000),
			new Segment(0.050000,100.882335,0.000000,100.885070,54.873813,14.931650,0.000000,0.000000),
			new Segment(0.050000,103.644691,0.000000,103.647426,55.620396,14.931650,0.000000,0.000000),
			new Segment(0.050000,106.444375,0.000000,106.447110,56.366978,14.931650,-0.000000,0.000000),
			new Segment(0.050000,109.281388,0.000000,109.284123,57.113561,14.931650,0.000000,0.000000),
			new Segment(0.050000,112.155731,0.000000,112.158466,57.860143,14.931650,0.000000,0.000000),
			new Segment(0.050000,115.067403,0.000000,115.070138,58.606726,14.931650,-0.000000,0.000000),
			new Segment(0.050000,118.016404,0.000000,118.019139,59.353308,14.931650,0.000000,0.000000),
			new Segment(0.050000,121.002734,0.000000,121.005469,60.099891,14.931650,0.000000,0.000000),
			new Segment(0.050000,124.026393,0.000000,124.029128,60.846473,14.931650,0.000000,0.000000),
			new Segment(0.050000,127.087381,0.000000,127.090116,61.593056,14.931650,-0.000000,0.000000),
			new Segment(0.050000,130.185698,0.000000,130.188433,62.339638,14.931650,0.000000,0.000000),
			new Segment(0.050000,133.321345,0.000000,133.324080,63.086221,14.931650,-0.000000,0.000000),
			new Segment(0.050000,136.491210,0.000000,136.493945,63.708373,12.443042,-49.772166,0.000000),
			new Segment(0.050000,139.689071,0.000000,139.691806,64.206095,9.954433,-49.772166,0.000000),
			new Segment(0.050000,142.908708,0.000000,142.911443,64.579386,7.465825,-49.772166,0.000000),
			new Segment(0.050000,146.143899,0.000000,146.146634,64.828247,4.977217,-49.772166,0.000000),
			new Segment(0.050000,149.388422,0.000000,149.391157,64.952677,2.488608,-49.772166,0.000000),
			new Segment(0.050000,152.634823,0.000000,152.637558,64.903349,-0.986560,-69.503375,0.000000),
			new Segment(0.050000,155.875646,0.000000,155.878381,64.729591,-3.475169,-49.772166,0.000000),
			new Segment(0.050000,159.104671,0.000000,159.107406,64.431402,-5.963777,-49.772166,0.000000),
			new Segment(0.050000,162.315676,0.000000,162.318411,64.008782,-8.452385,-49.772166,0.000000),
			new Segment(0.050000,165.502439,0.000000,165.505174,63.461733,-10.940994,-49.772166,0.000000),
			new Segment(0.050000,168.658738,0.000000,168.661473,62.790253,-13.429602,-49.772166,0.000000),
			new Segment(0.050000,171.779586,0.000000,171.782321,62.043670,-14.931650,-30.040957,0.000000),
			new Segment(0.050000,174.863105,0.000000,174.865840,61.297088,-14.931650,0.000000,0.000000),
			new Segment(0.050000,177.909295,0.000000,177.912030,60.550505,-14.931650,0.000000,0.000000),
			new Segment(0.050000,180.918156,0.000000,180.920891,59.803923,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,183.889687,0.000000,183.892422,59.057340,-14.931650,0.000000,0.000000),
			new Segment(0.050000,186.823890,0.000000,186.826625,58.310758,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,189.720763,0.000000,189.723498,57.564175,-14.931650,0.000000,0.000000),
			new Segment(0.050000,192.580307,0.000000,192.583042,56.817593,-14.931650,0.000000,0.000000),
			new Segment(0.050000,195.402522,0.000000,195.405257,56.071010,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,198.187408,0.000000,198.190143,55.324428,-14.931650,0.000000,0.000000),
			new Segment(0.050000,200.934965,0.000000,200.937700,54.577845,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,203.645193,0.000000,203.647928,53.831263,-14.931650,0.000000,0.000000),
			new Segment(0.050000,206.318092,0.000000,206.320827,53.084680,-14.931650,0.000000,0.000000),
			new Segment(0.050000,208.953661,0.000000,208.956396,52.338098,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,211.551901,0.000000,211.554636,51.591515,-14.931650,0.000000,0.000000),
			new Segment(0.050000,214.112813,0.000000,214.115548,50.844933,-14.931650,0.000000,0.000000),
			new Segment(0.050000,216.636395,0.000000,216.639130,50.098350,-14.931650,0.000000,0.000000),
			new Segment(0.050000,219.122648,0.000000,219.125383,49.351768,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,221.571571,0.000000,221.574306,48.605185,-14.931650,0.000000,0.000000),
			new Segment(0.050000,223.983166,0.000000,223.985901,47.858603,-14.931650,0.000000,0.000000),
			new Segment(0.050000,226.357432,0.000000,226.360167,47.112020,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,228.694368,0.000000,228.697103,46.365438,-14.931650,0.000000,0.000000),
			new Segment(0.050000,230.993975,0.000000,230.996710,45.618855,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,233.256254,0.000000,233.258989,44.872273,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,235.481203,0.000000,235.483938,44.125690,-14.931650,0.000000,0.000000),
			new Segment(0.050000,237.668823,0.000000,237.671558,43.379108,-14.931650,0.000000,0.000000),
			new Segment(0.050000,239.819113,0.000000,239.821848,42.632525,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,241.932075,0.000000,241.934810,41.885943,-14.931650,0.000000,0.000000),
			new Segment(0.050000,244.007708,0.000000,244.010443,41.139360,-14.931650,0.000000,0.000000),
			new Segment(0.050000,246.046011,0.000000,246.048746,40.392778,-14.931650,0.000000,0.000000),
			new Segment(0.050000,248.046986,0.000000,248.049721,39.646195,-14.931650,0.000000,0.000000),
			new Segment(0.050000,250.010631,0.000000,250.013366,38.899613,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,251.936947,0.000000,251.939682,38.153030,-14.931650,0.000000,0.000000),
			new Segment(0.050000,253.825934,0.000000,253.828669,37.406448,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,255.677592,0.000000,255.680327,36.659865,-14.931650,0.000000,0.000000),
			new Segment(0.050000,257.491920,0.000000,257.494655,35.913283,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,259.268920,0.000000,259.271655,35.166700,-14.931650,0.000000,0.000000),
			new Segment(0.050000,261.008590,0.000000,261.011325,34.420118,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,262.710932,0.000000,262.713667,33.673535,-14.931650,0.000000,0.000000),
			new Segment(0.050000,264.375944,0.000000,264.378679,32.926953,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,266.003627,0.000000,266.006362,32.180370,-14.931650,0.000000,0.000000),
			new Segment(0.050000,267.593981,0.000000,267.596716,31.433788,-14.931650,0.000000,0.000000),
			new Segment(0.050000,269.147006,0.000000,269.149741,30.687205,-14.931650,-0.000000,0.000000),
			new Segment(0.050000,270.662701,0.000000,270.665436,29.940623,-14.931650,0.000000,0.000000),
			new Segment(0.050000,272.141068,0.000000,272.143803,29.194040,-14.931650,0.000000,0.000000),
			new Segment(0.050000,273.581908,0.000835,273.584841,28.447458,-14.931650,-0.000000,0.020272),
			new Segment(0.050000,274.963821,0.224861,274.988549,27.700875,-14.931650,0.000000,0.274411),
			new Segment(0.050000,276.247722,0.689494,276.354928,26.954293,-14.931650,0.000000,0.406829),
			new Segment(0.050000,277.446485,1.262722,277.683978,26.207710,-14.931650,0.000000,0.478361),
			new Segment(0.050000,278.580467,1.881133,278.975699,25.461128,-14.931650,-0.000000,0.515948),
			new Segment(0.050000,279.665812,2.510015,280.230091,24.714545,-14.931650,0.000000,0.531285),
			new Segment(0.050000,280.714892,3.127002,281.447154,23.967963,-14.931650,0.000000,0.529287),
			new Segment(0.050000,281.737707,3.714864,282.626887,23.221380,-14.931650,-0.000000,0.511175),
			new Segment(0.050000,282.742944,4.257494,283.769292,22.474798,-14.931650,0.000000,0.475356),
			new Segment(0.050000,283.738560,4.736655,284.874367,21.728215,-14.931650,0.000000,0.417066),
			new Segment(0.050000,284.731594,5.128058,285.942113,20.981633,-14.931650,0.000000,0.326868),
			new Segment(0.050000,285.725818,5.395605,286.972530,20.235050,-14.931650,-0.000000,0.188087),
			new Segment(0.050000,286.713037,5.484868,287.965618,19.488468,-14.931650,0.000000,6.260110),
			new Segment(0.050000,287.652795,5.330628,288.921377,18.741885,-14.931650,0.000000,5.967596),
			new Segment(0.050000,288.466096,4.912357,289.839807,17.995303,-14.931650,-0.000000,5.650904),
			new Segment(0.050000,289.099262,4.303092,290.720907,17.248720,-14.931650,0.000000,5.395280),
			new Segment(0.050000,289.567726,3.602629,291.564679,16.502138,-14.931650,-0.000000,5.218791),
			new Segment(0.050000,289.913481,2.874596,292.371121,15.755556,-14.931650,0.000000,5.099318),
			new Segment(0.050000,290.172177,2.150533,293.140234,15.008973,-14.931650,-0.000000,5.015753),
			new Segment(0.050000,290.368515,1.445696,293.872019,14.262391,-14.931650,0.000000,4.954858),
			new Segment(0.050000,290.519140,0.767836,294.566474,13.515808,-14.931650,0.000000,4.908835),
			new Segment(0.050000,290.635480,0.121126,295.223599,12.769226,-14.931650,0.000000,4.873000),
			new Segment(0.050000,290.725618,-0.492060,295.843396,12.022643,-14.931650,-0.000000,4.844428),
			new Segment(0.050000,290.795444,-1.070314,296.425864,11.276061,-14.931650,0.000000,4.821217),
			new Segment(0.050000,290.849363,-1.612771,296.971002,10.529478,-14.931650,0.000000,4.802083),
			new Segment(0.050000,290.890744,-2.118886,297.478811,9.782896,-14.931650,-0.000000,4.786132),
			new Segment(0.050000,290.922214,-2.588309,297.949292,9.036313,-14.931650,0.000000,4.772725),
			new Segment(0.050000,290.945849,-3.020812,298.382443,8.289731,-14.931650,0.000000,4.761393),
			new Segment(0.050000,290.963317,-3.416247,298.778265,7.543148,-14.931650,-0.000000,4.751784),
			new Segment(0.050000,290.975961,-3.774516,299.136758,6.796566,-14.931650,0.000000,4.743633),
			new Segment(0.050000,290.984877,-4.095555,299.457921,6.049983,-14.931650,0.000000,4.736735),
			new Segment(0.050000,290.990956,-4.379324,299.741756,5.303401,-14.931650,-0.000000,4.730931),
			new Segment(0.050000,290.994927,-4.625797,299.988261,4.556818,-14.931650,0.000000,4.726098),
			new Segment(0.050000,290.997379,-4.834959,300.197438,3.810236,-14.931650,0.000000,4.722139),
			new Segment(0.050000,290.998782,-5.006801,300.369285,3.063653,-14.931650,-0.000000,4.718979),
			new Segment(0.050000,290.999505,-5.141317,300.503803,2.317071,-14.931650,0.000000,4.716562),
			new Segment(0.050000,290.999830,-5.239738,300.602225,1.619816,-13.945089,19.731209,4.714824),
			new Segment(0.050000,290.999953,-5.306408,300.668895,1.046992,-11.456481,49.772166,4.713660),
			new Segment(0.050000,290.999991,-5.347548,300.710035,0.598598,-8.967873,49.772166,4.712948),
			new Segment(0.050000,290.999999,-5.369379,300.731866,0.274635,-6.479265,49.772166,4.712572),
			new Segment(0.050000,291.000000,-5.378122,300.740609,0.075102,-3.990656,49.772166,4.712421),
			new Segment(0.050000,291.000000,-5.380000,300.742487,0.000000,-1.502048,49.772166,4.712389),
			new Segment(0.050000,291.000000,-5.380000,300.742487,0.000000,0.000000,30.040957,4.712389)
	};
	
}
