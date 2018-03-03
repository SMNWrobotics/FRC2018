package org.usfirst.frc1982.the2018Bot.commands;
import org.usfirst.frc1982.the2018Bot.Robot;
import org.usfirst.frc1982.the2018Bot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;

public class AutoSMidGSwLeft extends Command {
		
//		new Waypoint(0,0,0),
//		new Waypoint(105.5,62.0,0.0)
		
		EncoderFollower left;
		EncoderFollower right;
		
		@Override
		protected void initialize() {
			Trajectory trajectory;
			double max_velocity = 100; 
			
			trajectory = new Trajectory(segs); 
			
			EncoderFollower[] result = AutoUtil.getPath(trajectory, max_velocity);
			
			left = result[0];
			right = result[1];
			
			
			
		}
		
		@Override
		protected void execute() {
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
		
		@Override
		protected boolean isFinished() {
			// TODO Auto-generated method stub
			return false;
		}
		
		@Override
		protected void end() {
			
		}
		
		@Override
		protected void interrupted() {
			
		}
		
		private Segment[] segs = {
				new Segment(0.050000,0.003224,0.000000,0.004519,0.180766,3.615328,72.306554,0.000063),
				new Segment(0.050000,0.021300,0.000004,0.022596,0.542299,7.230655,72.306554,0.000419),
				new Segment(0.050000,0.061973,0.000038,0.063268,1.084598,10.845983,72.306554,0.001219),
				new Segment(0.050000,0.134279,0.000177,0.135575,1.807664,14.461311,72.306554,0.002643),
				new Segment(0.050000,0.247257,0.000602,0.248554,2.711496,18.076638,72.306554,0.004872),
				new Segment(0.050000,0.409944,0.001656,0.411244,3.796094,21.691966,72.306554,0.008088),
				new Segment(0.050000,0.626852,0.003877,0.628163,4.880692,21.691966,0.000000,0.012391),
				new Segment(0.050000,0.897970,0.007968,0.899313,5.965291,21.691966,0.000000,0.017791),
				new Segment(0.050000,1.223277,0.014814,1.224692,7.049889,21.691966,0.000000,0.024301),
				new Segment(0.050000,1.602735,0.025486,1.604302,8.134487,21.691966,0.000000,0.031939),
				new Segment(0.050000,2.036287,0.041242,2.038141,9.219086,21.691966,0.000000,0.040721),
				new Segment(0.050000,2.523845,0.063534,2.526210,10.303684,21.691966,-0.000000,0.050667),
				new Segment(0.050000,3.065284,0.094008,3.068509,11.388282,21.691966,0.000000,0.061795),
				new Segment(0.050000,3.660433,0.134512,3.665038,12.472880,21.691966,-0.000000,0.074125),
				new Segment(0.050000,4.309059,0.187096,4.315797,13.557479,21.691966,0.000000,0.087675),
				new Segment(0.050000,5.010859,0.254010,5.020786,14.642077,21.691966,-0.000000,0.102460),
				new Segment(0.050000,5.765442,0.337707,5.780005,15.726675,21.691966,0.000000,0.118492),
				new Segment(0.050000,6.572316,0.440837,6.593454,16.811274,21.691966,0.000000,0.135777),
				new Segment(0.050000,7.430872,0.566241,7.461132,17.895872,21.691966,0.000000,0.154314),
				new Segment(0.050000,8.340366,0.716936,8.383041,18.980470,21.691966,-0.000000,0.174095),
				new Segment(0.050000,9.299903,0.896099,9.359180,20.065069,21.691966,0.000000,0.195099),
				new Segment(0.050000,10.308425,1.107050,10.389548,21.149667,21.691966,0.000000,0.217291),
				new Segment(0.050000,11.364694,1.353212,11.474146,22.234265,21.691966,-0.000000,0.240624),
				new Segment(0.050000,12.467287,1.638086,12.612974,23.318864,21.691966,0.000000,0.265031),
				new Segment(0.050000,13.614591,1.965201,13.806033,24.403462,21.691966,0.000000,0.290427),
				new Segment(0.050000,14.804804,2.338070,15.053321,25.488060,21.691966,-0.000000,0.316708),
				new Segment(0.050000,16.035946,2.760132,16.354839,26.572658,21.691966,0.000000,0.343751),
				new Segment(0.050000,17.305875,3.234700,17.710586,27.657257,21.691966,-0.000000,0.371410),
				new Segment(0.050000,18.612319,3.764900,19.120564,28.741855,21.691966,0.000000,0.399523),
				new Segment(0.050000,19.952907,4.353612,20.584772,29.826453,21.691966,-0.000000,0.427913),
				new Segment(0.050000,21.325220,5.003424,22.103210,30.911052,21.691966,0.000000,0.456389),
				new Segment(0.050000,22.726836,5.716578,23.675877,31.995650,21.691966,-0.000000,0.484750),
				new Segment(0.050000,24.155395,6.494936,25.302775,33.080248,21.691966,0.000000,0.512794),
				new Segment(0.050000,25.608651,7.339955,26.983902,34.164847,21.691966,-0.000000,0.540314),
				new Segment(0.050000,27.084538,8.252667,28.719259,35.249445,21.691966,0.000000,0.567112),
				new Segment(0.050000,28.581221,9.233678,30.508846,36.334043,21.691966,-0.000000,0.592993),
				new Segment(0.050000,30.097155,10.283169,32.352664,37.418641,21.691966,0.000000,0.617776),
				new Segment(0.050000,31.631128,11.400910,34.250711,38.503240,21.691966,0.000000,0.641290),
				new Segment(0.050000,33.182301,12.586276,36.202988,39.587838,21.691966,0.000000,0.663380),
				new Segment(0.050000,34.750242,13.838270,38.209494,40.672436,21.691966,-0.000000,0.683901),
				new Segment(0.050000,36.334954,15.155539,40.270231,41.757035,21.691966,0.000000,0.702721),
				new Segment(0.050000,37.936896,16.536394,42.385198,42.841633,21.691966,-0.000000,0.719719),
				new Segment(0.050000,39.556999,17.978819,44.554394,43.926231,21.691966,-0.000000,0.734779),
				new Segment(0.050000,41.196688,19.480477,46.777821,45.010830,21.691966,0.000000,0.747790),
				new Segment(0.050000,42.854612,21.035595,49.050958,45.914662,18.076638,-72.306554,0.758624),
				new Segment(0.050000,44.526799,22.634805,51.364768,46.637727,14.461311,-72.306554,0.767149),
				new Segment(0.050000,46.209930,24.268255,53.710212,47.180026,10.845983,-72.306554,0.773271),
				new Segment(0.050000,47.901201,25.925725,56.078251,47.541559,7.230655,-72.306554,0.776917),
				new Segment(0.050000,49.597800,27.596372,58.459324,47.701344,3.195704,-80.699020,0.778041),
				new Segment(0.050000,51.293895,29.266034,60.839348,47.499597,-4.034951,-144.613107,0.776625),
				new Segment(0.050000,52.984014,30.920932,63.204765,47.117083,-7.650279,-72.306554,0.772692),
				new Segment(0.050000,54.665681,32.550617,65.546537,46.553802,-11.265606,-72.306554,0.766292),
				new Segment(0.050000,56.336038,34.144908,67.855626,45.809756,-14.880934,-72.306554,0.757498),
				new Segment(0.050000,70.590122,45.881430,86.341613,36.208156,-21.691966,0.000000,0.590099),
				new Segment(0.050000,79.200651,50.716126,96.227921,29.700566,-21.691966,-0.000000,0.424688),
				new Segment(0.050000,80.537354,51.298015,97.685835,28.615968,-21.691966,0.000000,0.396323),
				new Segment(0.050000,81.839636,51.821710,99.089518,27.531370,-21.691966,-0.000000,0.368255),
				new Segment(0.050000,83.105139,52.290125,100.438972,26.446771,-21.691966,0.000000,0.340662),
				new Segment(0.050000,84.331602,52.706409,101.734195,25.362173,-21.691966,-0.000000,0.313702),
				new Segment(0.050000,85.516901,53.073891,102.975189,24.277575,-21.691966,0.000000,0.287518),
				new Segment(0.050000,86.659077,53.396021,104.161953,23.192977,-21.691966,-0.000000,0.262233),
				new Segment(0.050000,87.756348,53.676317,105.294487,22.108378,-21.691966,0.000000,0.237947),
				new Segment(0.050000,88.807123,53.918306,106.372790,21.023780,-21.691966,0.000000,0.214744),
				new Segment(0.050000,89.810001,54.125481,107.396865,19.939182,-21.691966,-0.000000,0.192687),
				new Segment(0.050000,90.763765,54.301257,108.366709,18.854583,-21.691966,0.000000,0.171824),
				new Segment(0.050000,91.667377,54.448935,109.282323,17.769985,-21.691966,0.000000,0.152186),
				new Segment(0.050000,92.519959,54.571676,110.143707,16.685387,-21.691966,-0.000000,0.133793),
				new Segment(0.050000,93.320784,54.672478,110.950862,15.600788,-21.691966,0.000000,0.116653),
				new Segment(0.050000,94.069257,54.754159,111.703786,14.516190,-21.691966,0.000000,0.100766),
				new Segment(0.050000,94.764898,54.819347,112.402481,13.431592,-21.691966,-0.000000,0.086125),
				new Segment(0.050000,95.407326,54.870472,113.046945,12.346993,-21.691966,0.000000,0.072717),
				new Segment(0.050000,95.996248,54.909760,113.637180,11.262395,-21.691966,-0.000000,0.060526),
				new Segment(0.050000,96.531439,54.939239,114.173185,10.177797,-21.691966,0.000000,0.049536),
				new Segment(0.050000,97.012732,54.960731,114.654960,9.093199,-21.691966,-0.000000,0.039725),
				new Segment(0.050000,97.440008,54.975861,115.082505,8.008600,-21.691966,0.000000,0.031076),
				new Segment(0.050000,97.813183,54.986058,115.455820,6.924002,-21.691966,-0.000000,0.023570),
				new Segment(0.050000,98.132202,54.992560,115.774905,5.839404,-21.691966,0.000000,0.017188),
				new Segment(0.050000,98.397028,54.996413,116.039760,4.754805,-21.691966,0.000000,0.011916),
				new Segment(0.050000,98.607643,54.998483,116.250385,3.670207,-21.691966,-0.000000,0.007740),
				new Segment(0.050000,98.764560,54.999454,116.407305,2.606590,-21.272343,8.392466,0.004638),
				new Segment(0.050000,98.872818,54.999841,116.515563,1.723739,-17.657015,72.306554,0.002503),
				new Segment(0.050000,98.941453,54.999966,116.584198,1.021655,-14.041687,72.306554,0.001152),
				new Segment(0.050000,98.979502,54.999996,116.622248,0.500337,-10.426360,72.306554,0.000403),
				new Segment(0.050000,98.996005,55.000000,116.638751,0.159785,-6.811032,72.306554,0.000079),
				new Segment(0.050000,99.000000,55.000000,116.642746,0.000000,-3.195704,72.306554,6.283185),
				new Segment(0.050000,99.000000,55.000000,116.642746,0.000000,0.000000,63.914087,6.283185)
//				new Segment(0.050000,0.001658,0.000000,0.003077,0.123097,2.461943,49.238854,0.000031),
//				new Segment(0.050000,0.013968,0.000002,0.015387,0.369291,4.923885,49.238854,0.000258),
//				new Segment(0.050000,0.041665,0.000016,0.043084,0.738583,7.385828,49.238854,0.000770),
//				new Segment(0.050000,0.090903,0.000076,0.092323,1.230971,9.847771,49.238854,0.001680),
//				new Segment(0.050000,0.167839,0.000260,0.169259,1.846457,12.309713,49.238854,0.003104),
//				new Segment(0.050000,0.278625,0.000718,0.280046,2.585040,14.771656,49.238854,0.005158),
//				new Segment(0.050000,0.426339,0.001683,0.427763,3.323623,14.771656,0.000000,0.007903),
//				new Segment(0.050000,0.610976,0.003459,0.612408,4.062205,14.771656,-0.000000,0.011345),
//				new Segment(0.050000,0.832531,0.006432,0.833983,4.800788,14.771656,0.000000,0.015490),
//				new Segment(0.050000,1.090993,0.011063,1.092487,5.539371,14.771656,0.000000,0.020347),
//				new Segment(0.050000,1.386346,0.017897,1.387920,6.277954,14.771656,-0.000000,0.025923),
//				new Segment(0.050000,1.718568,0.027558,1.720282,7.016537,14.771656,0.000000,0.032230),
//				new Segment(0.050000,2.087622,0.040758,2.089574,7.755119,14.771656,0.000000,0.039278),
//				new Segment(0.050000,2.493463,0.058291,2.495794,8.493702,14.771656,0.000000,0.047078),
//				new Segment(0.050000,2.936027,0.081039,2.938944,9.232285,14.771656,-0.000000,0.055644),
//				new Segment(0.050000,3.415232,0.109975,3.419023,9.970868,14.771656,0.000000,0.064987),
//				new Segment(0.050000,3.930970,0.146160,3.936031,10.709451,14.771656,0.000000,0.075120),
//				new Segment(0.050000,4.483106,0.190749,4.489968,11.448033,14.771656,0.000000,0.086054),
//				new Segment(0.050000,5.071475,0.244985,5.080834,12.186616,14.771656,0.000000,0.097802),
//				new Segment(0.050000,5.695869,0.310208,5.708630,12.925199,14.771656,-0.000000,0.110373),
//				new Segment(0.050000,6.356039,0.387847,6.373354,13.663782,14.771656,0.000000,0.123776),
//				new Segment(0.050000,7.051685,0.479423,7.075008,14.402365,14.771656,0.000000,0.138017),
//				new Segment(0.050000,7.782451,0.586545,7.813591,15.140947,14.771656,-0.000000,0.153100),
//				new Segment(0.050000,8.547918,0.710904,8.589103,15.879530,14.771656,-0.000000,0.169023),
//				new Segment(0.050000,9.347600,0.854273,9.401544,16.618113,14.771656,0.000000,0.185783),
//				new Segment(0.050000,10.180932,1.018494,10.250914,17.356696,14.771656,0.000000,0.203370),
//				new Segment(0.050000,11.047271,1.205472,11.137213,18.095279,14.771656,-0.000000,0.221767),
//				new Segment(0.050000,11.945887,1.417166,12.060442,18.833862,14.771656,0.000000,0.240953),
//				new Segment(0.050000,12.875960,1.655571,13.020599,19.572444,14.771656,0.000000,0.260896),
//				new Segment(0.050000,13.836577,1.922706,14.017686,20.311027,14.771656,0.000000,0.281560),
//				new Segment(0.050000,14.826734,2.220596,15.051702,21.049610,14.771656,-0.000000,0.302896),
//				new Segment(0.050000,15.845333,2.551252,16.122647,21.788193,14.771656,0.000000,0.324849),
//				new Segment(0.050000,16.891190,2.916651,17.230521,22.526776,14.771656,-0.000000,0.347352),
//				new Segment(0.050000,17.963040,3.318714,18.375325,23.265358,14.771656,0.000000,0.370332),
//				new Segment(0.050000,19.059547,3.759282,19.557057,24.003941,14.771656,0.000000,0.393705),
//				new Segment(0.050000,20.179317,4.240096,20.775719,24.742524,14.771656,-0.000000,0.417378),
//				new Segment(0.050000,21.320913,4.762774,22.031310,25.481107,14.771656,0.000000,0.441253),
//				new Segment(0.050000,22.482874,5.328790,23.323829,26.219690,14.771656,0.000000,0.465224),
//				new Segment(0.050000,23.663735,5.939460,24.653279,26.958272,14.771656,0.000000,0.489183),
//				new Segment(0.050000,24.862050,6.595922,26.019657,27.696855,14.771656,0.000000,0.513017),
//				new Segment(0.050000,26.076414,7.299129,27.422964,28.435438,14.771656,-0.000000,0.536612),
//				new Segment(0.050000,27.305488,8.049837,28.863201,29.174021,14.771656,0.000000,0.559855),
//				new Segment(0.050000,28.548023,8.848606,30.340366,29.912604,14.771656,0.000000,0.582635),
//				new Segment(0.050000,29.802878,9.695794,31.854461,30.651186,14.771656,0.000000,0.604846),
//				new Segment(0.050000,31.069045,10.591563,33.405485,31.389769,14.771656,0.000000,0.626385),
//				new Segment(0.050000,32.345667,11.535883,34.993438,32.128352,14.771656,-0.000000,0.647155),
//				new Segment(0.050000,33.632052,12.528541,36.618320,32.866935,14.771656,0.000000,0.667065),
//				new Segment(0.050000,34.927688,13.569147,38.280131,33.605518,14.771656,-0.000000,0.686033),
//				new Segment(0.050000,36.232257,14.657146,39.978872,34.344100,14.771656,0.000000,0.703980),
//				new Segment(0.050000,37.545639,15.791829,41.714541,35.082683,14.771656,0.000000,0.720835),
//				new Segment(0.050000,38.867925,16.972338,43.487140,35.821266,14.771656,-0.000000,0.736533),
//				new Segment(0.050000,40.199423,18.197674,45.296668,36.559849,14.771656,0.000000,0.751013),
//				new Segment(0.050000,41.540657,19.466707,47.143125,37.298432,14.771656,0.000000,0.764216),
//				new Segment(0.050000,42.892379,20.778175,49.026511,38.037014,14.771656,-0.000000,0.776087),
//				new Segment(0.050000,44.255572,22.130685,50.946826,38.775597,14.771656,0.000000,0.786573),
//				new Segment(0.050000,45.631451,23.522711,52.904071,39.514180,14.771656,0.000000,0.795618),
//				new Segment(0.050000,47.019335,24.950374,54.895167,40.129666,12.309713,-49.238854,0.803156),
//				new Segment(0.050000,48.416722,26.407365,56.913960,40.622054,9.847771,-49.238854,0.809118),
//				new Segment(0.050000,49.821419,27.887158,58.954295,40.991346,7.385828,-49.238854,0.813453),
//				new Segment(0.050000,51.231479,29.383059,61.010017,41.237540,4.923885,-49.238854,0.816122),
//				new Segment(0.050000,52.645130,30.888260,63.074972,41.360637,2.461943,-49.238854,0.817100),
//				new Segment(0.050000,54.058901,32.393970,65.140380,41.255699,-2.098767,-91.214184,0.816375),
//				new Segment(0.050000,55.469330,33.891395,67.197464,41.027663,-4.560709,-49.238854,0.813956),
//				new Segment(0.050000,56.874662,35.373714,69.240069,40.676531,-7.022652,-49.238854,0.809868),
//				new Segment(0.050000,58.272979,36.834216,71.262040,40.202301,-9.484595,-49.238854,0.804146),
//				new Segment(0.050000,59.662128,38.266346,73.257221,39.604974,-11.946537,-49.238854,0.796840),
//				new Segment(0.050000,61.039655,39.663763,75.219460,38.884550,-14.408480,-49.238854,0.788017),
//				new Segment(0.050000,62.404610,41.022229,77.145223,38.145967,-14.771656,-7.263523,0.777744),
//				new Segment(0.050000,63.757927,42.339873,79.034056,37.407384,-14.771656,-0.000000,0.766077),
//				new Segment(0.050000,65.100622,43.615283,80.885961,36.668802,-14.771656,0.000000,0.753070),
//				new Segment(0.050000,66.433481,44.847179,82.700936,35.930219,-14.771656,-0.000000,0.738778),
//				new Segment(0.050000,67.757063,46.034410,84.478983,35.191636,-14.771656,0.000000,0.723260),
//				new Segment(0.050000,69.071707,47.175957,86.220100,34.453053,-14.771656,0.000000,0.706574),
//				new Segment(0.050000,70.377535,48.270942,87.924288,33.714470,-14.771656,-0.000000,0.688787),
//				new Segment(0.050000,71.674460,49.318633,89.591547,32.975888,-14.771656,0.000000,0.669968),
//				new Segment(0.050000,72.962192,50.318451,91.221877,32.237305,-14.771656,0.000000,0.650195),
//				new Segment(0.050000,74.240247,51.269981,92.815278,31.498722,-14.771656,-0.000000,0.629549),
//				new Segment(0.050000,75.507960,52.172984,94.371749,30.760139,-14.771656,0.000000,0.608121),
//				new Segment(0.050000,76.764497,53.027401,95.891292,30.021556,-14.771656,-0.000000,0.586006),
//				new Segment(0.050000,78.008872,53.833366,97.373905,29.282974,-14.771656,0.000000,0.563306),
//				new Segment(0.050000,79.239963,54.591208,98.819589,28.544391,-14.771656,0.000000,0.540126),
//				new Segment(0.050000,80.456535,55.301455,100.228344,27.805808,-14.771656,-0.000000,0.516579),
//				new Segment(0.050000,81.657261,55.964836,101.600170,27.067225,-14.771656,0.000000,0.492776),
//				new Segment(0.050000,82.840746,56.582273,102.935066,26.328642,-14.771656,0.000000,0.468830),
//				new Segment(0.050000,84.005547,57.154879,104.233034,25.590060,-14.771656,-0.000000,0.444856),
//				new Segment(0.050000,85.150202,57.683941,105.494072,24.851477,-14.771656,0.000000,0.420961),
//				new Segment(0.050000,86.273247,58.170914,106.718182,24.112894,-14.771656,-0.000000,0.397253),
//				new Segment(0.050000,87.373242,58.617393,107.905362,23.374311,-14.771656,0.000000,0.373832),
//				new Segment(0.050000,88.448784,59.025103,109.055613,22.635728,-14.771656,-0.000000,0.350790),
//				new Segment(0.050000,89.498529,59.395871,110.168935,21.897146,-14.771656,-0.000000,0.328212),
//				new Segment(0.050000,90.521200,59.731607,111.245327,21.158563,-14.771656,0.000000,0.306174),
//				new Segment(0.050000,91.515600,60.034282,112.284791,20.419980,-14.771656,-0.000000,0.284744),
//				new Segment(0.050000,92.480619,60.305901,113.287325,19.681397,-14.771656,0.000000,0.263978),
//				new Segment(0.050000,93.415238,60.548490,114.252931,18.942814,-14.771656,0.000000,0.243926),
//				new Segment(0.050000,94.318532,60.764068,115.181607,18.204232,-14.771656,0.000000,0.224626),
//				new Segment(0.050000,95.189665,60.954636,116.073354,17.465649,-14.771656,0.000000,0.206111),
//				new Segment(0.050000,96.027896,61.122158,116.928172,16.727066,-14.771656,0.000000,0.188403),
//				new Segment(0.050000,96.832568,61.268547,117.746060,15.988483,-14.771656,0.000000,0.171520),
//				new Segment(0.050000,97.603105,61.395656,118.527020,15.249900,-14.771656,-0.000000,0.155473),
//				new Segment(0.050000,98.339010,61.505269,119.271050,14.511318,-14.771656,0.000000,0.140266),
//				new Segment(0.050000,99.039853,61.599088,119.978152,13.772735,-14.771656,-0.000000,0.125900),
//				new Segment(0.050000,99.705271,61.678736,120.648324,13.034152,-14.771656,-0.000000,0.112373),
//				new Segment(0.050000,100.334954,61.745746,121.281567,12.295569,-14.771656,0.000000,0.099679),
//				new Segment(0.050000,100.928646,61.801561,121.877881,11.556986,-14.771656,-0.000000,0.087810),
//				new Segment(0.050000,101.486136,61.847533,122.437266,10.818404,-14.771656,0.000000,0.076756),
//				new Segment(0.050000,102.007250,61.884921,122.959721,10.079821,-14.771656,0.000000,0.066505),
//				new Segment(0.050000,102.491848,61.914893,123.445248,9.341238,-14.771656,0.000000,0.057046),
//				new Segment(0.050000,102.939822,61.938523,123.893845,8.602655,-14.771656,0.000000,0.048366),
//				new Segment(0.050000,103.351083,61.956797,124.305513,7.864072,-14.771656,-0.000000,0.040453),
//				new Segment(0.050000,103.725566,61.970611,124.680252,7.125489,-14.771656,0.000000,0.033294),
//				new Segment(0.050000,104.063223,61.980771,125.018062,6.386907,-14.771656,-0.000000,0.026878),
//				new Segment(0.050000,104.364016,61.988002,125.318943,5.648324,-14.771656,0.000000,0.021194),
//				new Segment(0.050000,104.627922,61.992941,125.582894,4.909741,-14.771656,-0.000000,0.016232),
//				new Segment(0.050000,104.854921,61.996143,125.809917,4.171158,-14.771656,0.000000,0.011982),
//				new Segment(0.050000,105.045005,61.998083,126.000010,3.432575,-14.771656,0.000000,0.008437),
//				new Segment(0.050000,105.198165,61.999157,126.153174,2.693993,-14.771656,-0.000000,0.005589),
//				new Segment(0.050000,105.314399,61.999682,126.269410,1.955410,-14.771656,0.000000,0.003433),
//				new Segment(0.050000,105.396328,61.999901,126.351339,1.321765,-12.672890,41.975331,0.001916),
//				new Segment(0.050000,105.449652,61.999977,126.404663,0.811218,-10.210947,49.238854,0.000930),
//				new Segment(0.050000,105.480527,61.999996,126.435538,0.423768,-7.749004,49.238854,0.000360),
//				new Segment(0.050000,105.495107,62.000000,126.450118,0.159415,-5.287062,49.238854,0.000090),
//				new Segment(0.050000,105.499546,62.000000,126.454557,0.018159,-2.825119,49.238854,0.000008),
//				new Segment(0.050000,105.500000,62.000000,126.455011,0.000000,-0.363176,49.238854,6.283185),
//				new Segment(0.050000,105.500000,62.000000,126.455011,0.000000,0.000000,7.263523,6.283185)
		};
}
