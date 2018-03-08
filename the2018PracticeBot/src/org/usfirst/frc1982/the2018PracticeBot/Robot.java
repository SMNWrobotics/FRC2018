package org.usfirst.frc1982.the2018PracticeBot;

import org.frc1982.common.AutoChooser;
import org.frc1982.common.Goal;
import org.frc1982.common.Position;
import org.frc1982.common.Side;
import org.usfirst.frc1982.the2018PracticeBot.commands.AutoUtil;
import org.usfirst.frc1982.the2018PracticeBot.commands.AutoWaypointTest;
import org.usfirst.frc1982.the2018PracticeBot.commands._AutoGeneral;
import org.usfirst.frc1982.the2018PracticeBot.subsystems.Claw;
import org.usfirst.frc1982.the2018PracticeBot.subsystems.Drive;
import org.usfirst.frc1982.the2018PracticeBot.subsystems.DrivePneumatics;
import org.usfirst.frc1982.the2018PracticeBot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    Command autonomousCommand;
    public static ADIS16448_IMU gyro;
    public static boolean gyroValid;
    public static boolean useGyroForAuto = false;
    public static final int max_velocity = 100;
    
    
    public static OI oi;
    public static Drive drive;
    public static DrivePneumatics pneumatics;
    public static Claw claw;
    public static Elevator elevator;


    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        drive = new Drive();
        pneumatics = new DrivePneumatics();
        claw = new Claw();
        elevator = new Elevator();
        
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

//        chooser.addDefault("Auto Do Nothing", new AutoDoNothing());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
//        chooser.addObject("Autonomous Test", new _AutoTest());
//        chooser.addObject("Auto Waypoint Test", new AutoWaypointTest());
//        chooser.addObject("Mid to Left Switch", new _AutoSMidGSwLeft());
//        chooser.addObject("Mid to Right Switch", new _AutoSMidGSwRight());
//        chooser.addObject("Left to Left Switch", new _AutoSLeftGSwLeft());
//        SmartDashboard.putData("Auto mode", chooser);
        
        gyro = new ADIS16448_IMU();
        
//        gyro.calibrate(); //already done in constructor
        gyro.reset();
        
        RobotMap.driveRight.setNeutralMode(NeutralMode.Brake);
        RobotMap.driveLeft.setNeutralMode(NeutralMode.Brake);
        
//        PowerDistributionPanel pdp = new PowerDistributionPanel();
//        SmartDashboard.putData(pdp);
//        
//        SmartDashboard.putBoolean("Only Generate Autonomous", true);
        
        CameraServer.getInstance().startAutomaticCapture();
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){
    }
    
    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }
    
    private Goal goal;
    private Side targetSide;
    
    public boolean parseGameData(String GameData) {
    	//delete once wyatt makes the thing
    	char startingPos = 'L';
    	boolean switchSwSc = false;
    	boolean Sw = true;
    	boolean Sc = false;
    	boolean Line = true;
    	//if the gamedata actually has something useful:
    	if (GameData.length() > 0) {
    		if (!switchSwSc) { //if we are following the normal priority list:
    			if (Sc && GameData.charAt(1) == startingPos) {
    				goal = Goal.SCALE;
    				
    			} else if (Sw && GameData.charAt(0) == startingPos) {
    				goal = Goal.SWITCH;
    			} else if (Line) {
    				goal = Goal.LINE;
    			} else {
    				goal = Goal.NOTHING;
    			}
    		} else { //if we are switching the switch (heh) and scale position in the priority list
    			if (Sw && GameData.charAt(0) == startingPos) {
    				goal = Goal.SWITCH;
    			} else if (Sc && GameData.charAt(1) == startingPos) {
    				goal = Goal.SCALE;
    			} else if (Line) {
    				goal = Goal.LINE;
    			} else {
    				goal = Goal.NOTHING;
    			}
    			
    		}
    	}
    	return false;
    }
    
    
    @Override
    public void autonomousInit() {
    	RobotMap.pneumaticsGearShifter.set(DoubleSolenoid.Value.kReverse);
        
        autonomousCommand = new _AutoGeneral(AutoChooser.getAutoPath(Position.MID, Goal.NOTHING, "LRL"), Goal.SWITCH);
    	
        if (autonomousCommand != null) {
        	System.out.println("auto Should begin running now");
        	autonomousCommand.start();
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        RobotMap.pneumaticsGearShifter.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }
}
