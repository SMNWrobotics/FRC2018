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
		double max_velocity = 200;
		
		//trajectory for the robot to follow.  Made in one of two ways depending on the boolean generateNew
		Trajectory trajectory;
		
		//if we want to generate a new trajectory (for instance to record the points, etc.)
		if (generateNew) {
			Trajectory.Config config //make new configuration variable, with the following properties
	        = new Trajectory.Config( Trajectory.FitMethod.HERMITE_CUBIC
	        		               , Trajectory.Config.SAMPLES_HIGH
	        		               , 0.05 //time delta between points (seconds)
	        		               , max_velocity //max_velocity inches / second
	        		               , 100 //max_acceleration inches / second^2
	        		               , 80.0  //max_jerk inches / second^3
	        		               );
			//waypoint comments: x is forwards, y is sideways
			//negative angle is rotate counterclockwise, + is rotate clockwise
			//list of waypoints to go through
			Waypoint[] points = new Waypoint[] {
					new Waypoint(0,0,0),
					new Waypoint(99,55.0,0.0)
					
//				//test output from my processing program: (should be for left side of switch)
//				new Waypoint(0.0,0.0,1.5707964),
//				new Waypoint(89.96737,69.41397,0.3507974),
//				new Waypoint(124.95364,119.913956,1.5407963),
//				new Waypoint(105.46088,148.91397, 2.7207952),
//				new Waypoint(94.46495,151.91396, 3.140795)
				
					//not sure which one this was (last used the last saturday in January? (before robot taken away))
//				new Waypoint(0.0,0.0,0.0),
//				new Waypoint(72.0,0.0,0.0),
//				new Waypoint(145.0,0.0,0.0),
//				new Waypoint(200.0,0.0,0.0),
//				new Waypoint(250,-2,Pathfinder.d2r(-45)),
//				new Waypoint(272.0,-3.0,Pathfinder.d2r(-75)),
//				new Waypoint(291.0,-5.0,Pathfinder.d2r(-90))
				
					//WORKS!!!: (untested, but graph looks nice) (TODO: make it move only 5 inch. forward rather than 7!)
//				new Waypoint(0.0,0.0,0.0),
//				new Waypoint(200.0,0.0,0.0),
//				new Waypoint(266.0,0.0,0.0),
//				new Waypoint(286.0,10.0,Pathfinder.d2r(-45)),
//				new Waypoint(291.0,-5.0,Pathfinder.d2r(-90))
////				new Waypoint(0.0,0.0,0.0),
////				new Waypoint(271.0,0.0,0.0),
////				new Waypoint(291.0,-5.0,Pathfinder.d2r(-45)),
////				new Waypoint(291.0,-7.0,Pathfinder.d2r(-90))
				
				//middle of above code:

//				new Waypoint(250.0,0.0,0.0),
//				new Waypoint(275.0,-.5,Pathfinder.d2r(-40)),
//				new Waypoint(285.0,-1.0,Pathfinder.d2r(-60)),
				
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
	    
		EncoderFollower[] followers = AutoUtil.getPath(trajectory, max_velocity);
		
		left = followers[0];
		right = followers[1];
		
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
	}
	
	@Override
	protected void execute() {
		if (!generateOnly) {
			int lpos = (int) Math.round(RobotMap.driveLeft.getSelectedSensorPosition(0));
			int rpos = (int) Math.round(RobotMap.driveRight.getSelectedSensorPosition(0));
			double l = left.calculate(lpos);
		    double r = right.calculate(rpos);
		    
		    double turn = 0.0;
		    double gyro_heading = -Robot.gyro.getAngleZ()/4; // Assuming the gyro is giving a value in degrees
		    if (true) {//Robot.gyroValid) {
			    double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
			    
			    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
			    turn = 0.8 * (-1.0/80.0) * angleDifference;
			    
			    SmartDashboard.putNumber("Gyro Angle", gyro_heading);
			    SmartDashboard.putNumber("Desired Heading", desired_heading);
		    }
		    System.out.println("Lencoder: " + lpos + " Rencoder: " + rpos + " Left output: " + l + " Right output: " + r);// + " desired heading: " + desired_heading);
		    
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
			new Segment(0.050000,0.003495,0.000000,0.004791,0.191637,3.832733,76.654656,0.000069),
			new Segment(0.050000,0.022659,0.000005,0.023955,0.574910,7.665466,76.654656,0.000446),
			new Segment(0.050000,0.065777,0.000043,0.067073,1.149820,11.498198,76.654656,0.001294),
			new Segment(0.050000,0.142432,0.000200,0.143727,1.916366,15.330931,76.654656,0.002804),
			new Segment(0.050000,0.262204,0.000677,0.263500,2.874550,19.163664,76.654656,0.005167),
			new Segment(0.050000,0.434672,0.001862,0.435973,4.024369,22.996397,76.654656,0.008578),
			new Segment(0.050000,0.669413,0.004422,0.670728,5.365826,26.829130,76.654656,0.013237),
			new Segment(0.050000,0.975991,0.009417,0.977347,6.898919,30.661862,76.654656,0.019349),
			new Segment(0.050000,1.363949,0.018432,1.365411,8.623649,34.494595,76.654656,0.027127),
			new Segment(0.050000,1.842794,0.033739,1.844503,10.540015,38.327328,76.654656,0.036795),
			new Segment(0.050000,2.421964,0.058474,2.424203,12.648018,42.160061,76.654656,0.048583),
			new Segment(0.050000,3.110782,0.096844,3.114095,14.947658,45.992794,76.654656,0.062734),
			new Segment(0.050000,3.918392,0.154366,3.923760,17.438934,49.825526,76.654656,0.079501),
			new Segment(0.050000,4.848887,0.237636,4.857989,19.930211,49.825526,-0.000000,0.099037),
			new Segment(0.050000,5.901235,0.354073,5.916781,22.421487,49.825526,0.000000,0.121391),
			new Segment(0.050000,7.073956,0.512134,7.100138,24.912763,49.825526,0.000000,0.146591),
			new Segment(0.050000,8.365003,0.721275,8.408058,27.404040,49.825526,0.000000,0.174633),
			new Segment(0.050000,9.771641,0.991864,9.840541,29.895316,49.825526,-0.000000,0.205467),
			new Segment(0.050000,11.290327,1.335029,11.397589,32.386592,49.825526,0.000000,0.238979),
			new Segment(0.050000,12.916619,1.762446,13.079201,34.877869,49.825526,-0.000000,0.274980),
			new Segment(0.050000,14.645121,2.286046,14.885376,37.369145,49.825526,-0.000000,0.313189),
			new Segment(0.050000,16.469493,2.917649,16.816115,39.860421,49.825526,0.000000,0.353224),
			new Segment(0.050000,18.382556,3.668551,18.871418,42.351697,49.825526,0.000000,0.394608),
			new Segment(0.050000,20.376492,4.549083,21.051285,44.842974,49.825526,-0.000000,0.436771),
			new Segment(0.050000,22.443139,5.568190,23.355716,47.334250,49.825526,0.000000,0.479074),
			new Segment(0.050000,24.570229,6.730687,25.779919,49.633890,45.992794,-76.654656,0.520758),
			new Segment(0.050000,26.742324,8.036173,28.314314,51.741893,42.160061,-76.654656,0.561007),
			new Segment(0.050000,28.945737,9.480948,30.949317,53.658259,38.327328,-76.654656,0.599087),
			new Segment(0.050000,31.168895,11.058301,33.675349,55.382989,34.494595,-76.654656,0.634376),
			new Segment(0.050000,33.402530,12.758926,36.482825,56.916082,30.661862,-76.654656,0.666374),
			new Segment(0.050000,35.639699,14.571418,39.362166,58.257539,26.829130,-76.654656,0.694706),
			new Segment(0.050000,37.875660,16.482750,42.303788,59.407358,22.996397,-76.654656,0.719107),
			new Segment(0.050000,40.107655,18.478710,45.298111,60.365542,19.163664,-76.654656,0.739401),
			new Segment(0.050000,42.334618,20.544256,48.335552,61.132088,15.330931,-76.654656,0.755479),
			new Segment(0.050000,44.556860,22.663792,51.406529,61.706998,11.498198,-76.654656,0.767281),
			new Segment(0.050000,46.775743,24.821364,54.501460,62.090271,7.665466,-76.654656,0.774771),
			new Segment(0.050000,48.993363,27.000797,57.610765,62.281908,3.832733,-76.654656,0.777932),
			new Segment(0.050000,51.210381,29.183962,60.722256,62.177731,-2.083544,-118.325537,0.776754),
			new Segment(0.050000,53.427533,31.352708,63.823747,61.881917,-5.916277,-76.654656,0.771244),
			new Segment(0.050000,55.647135,33.490803,66.905657,61.394467,-9.749010,-76.654656,0.761417),
			new Segment(0.050000,57.870915,35.582192,69.958403,60.715379,-13.581742,-76.654656,0.747299),
			new Segment(0.050000,60.099694,37.611106,72.972404,59.844656,-17.414475,-76.654656,0.728936),
			new Segment(0.050000,62.333067,39.562232,75.938077,58.782295,-21.247208,-76.654656,0.706413),
			new Segment(0.050000,64.569079,41.420955,78.845842,57.528298,-25.079941,-76.654656,0.679872),
			new Segment(0.050000,66.803924,43.173669,81.686116,56.082664,-28.912674,-76.654656,0.649536),
			new Segment(0.050000,69.031686,44.808182,84.449318,54.445394,-32.745406,-76.654656,0.615728),
			new Segment(0.050000,71.244166,46.314179,87.125865,52.616487,-36.578139,-76.654656,0.578887),
			new Segment(0.050000,73.430836,47.683712,89.706176,50.595944,-40.410872,-76.654656,0.539581),
			new Segment(0.050000,75.578948,48.911665,92.180668,48.383763,-44.243605,-76.654656,0.498494),
			new Segment(0.050000,77.673819,49.996104,94.539761,45.979946,-48.076338,-76.654656,0.456408),
			new Segment(0.050000,79.701660,50.939510,96.776476,43.488670,-49.825526,-34.983775,0.414115),
			new Segment(0.050000,81.652362,51.749012,98.888628,40.997394,-49.825526,-0.000000,0.372314),
			new Segment(0.050000,83.517912,52.434348,100.876216,38.506118,-49.825526,-0.000000,0.331608),
			new Segment(0.050000,85.290806,53.006416,102.739240,36.014841,-49.825526,0.000000,0.292519),
			new Segment(0.050000,86.964304,53.476844,104.477700,33.523565,-49.825526,-0.000000,0.255474),
			new Segment(0.050000,88.532571,53.857553,106.091596,31.032289,-49.825526,0.000000,0.220799),
			new Segment(0.050000,89.990728,54.160371,107.580929,28.541012,-49.825526,-0.000000,0.188724),
			new Segment(0.050000,91.334829,54.396704,108.945698,26.049736,-49.825526,0.000000,0.159397),
			new Segment(0.050000,92.561780,54.577285,110.185902,23.558460,-49.825526,-0.000000,0.132894),
			new Segment(0.050000,93.669231,54.712001,111.301544,21.067183,-49.825526,0.000000,0.109240),
			new Segment(0.050000,94.655456,54.809772,112.292621,18.575907,-49.825526,-0.000000,0.088420),
			new Segment(0.050000,95.519228,54.878492,113.159134,16.084631,-49.825526,0.000000,0.070393),
			new Segment(0.050000,96.262311,54.925152,113.903688,13.697532,-47.741982,41.670881,0.055052),
			new Segment(0.050000,96.891553,54.955765,114.533678,11.502069,-43.909250,76.654656,0.042189),
			new Segment(0.050000,97.416202,54.975115,115.058686,9.498243,-40.076517,76.654656,0.031557)
	};
	
}
