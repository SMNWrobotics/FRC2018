// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc1982.the2014Bot;

import java.util.Calendar;

import org.usfirst.frc1982.the2014Bot.commands._AutoDivert;
import org.usfirst.frc1982.the2014Bot.commands._AutoDoNothing;
import org.usfirst.frc1982.the2014Bot.commands._AutoSwitch;
import org.usfirst.frc1982.the2014Bot.subsystems.DriveTrain;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
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
    SendableChooser<Command> autoProgramChoice = new SendableChooser<>();
    SendableChooser<Position> startingPosition = new SendableChooser<>();
    ADIS16448_IMU gyro;

    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
//    public static DriveTrain driveTrain;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
//        driveTrain = new DriveTrain();

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        autoProgramChoice.addDefault("AutoDoNothing", new _AutoDoNothing());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        autoProgramChoice.addObject("AutoDivert", new _AutoDivert());
        autoProgramChoice.addObject( "AutoSwitch", new _AutoSwitch());
        
        SmartDashboard.putData("Auto mode", autoProgramChoice);
        
        startingPosition.addDefault( Position.ONE.name(), Position.ONE);
        startingPosition.addObject( Position.TWO.name(), Position.TWO);
        startingPosition.addObject( Position.THREE.name(), Position.THREE);
        
        SmartDashboard.putData("Starting Position", startingPosition);
        gyro = new ADIS16448_IMU();
////        gyro.calibrate();
//        gyro.reset();
//        
//        gyro.calibrate( 10.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 10.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 10.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 10.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 10.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 10.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 20.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 30.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 40.0 );
//        gyro.reset();
//        
//        gyro.calibrate( 50.0 );
//        gyro.reset();
        
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution( ImageResolution.MEDIUM.getWidth(), ImageResolution.MEDIUM.getHeight());
        camera.setFPS(1);
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){
    	start = Calendar.getInstance().getTimeInMillis() / 1000;
    }

    private int i = 0;
    private long start;
    
    @Override
    public void disabledPeriodic() {
//    	i++;
//    	if (i % 50 == 0) {
//        Scheduler.getInstance().run();
//        long elapsedTime = Calendar.getInstance().getTimeInMillis()/1000-start;
//        System.out.print( Integer.toString(i) + " :: Roll angle: " + Math.round(gyro.getAngleX()));
//        System.out.print(" Pitch angle: " + Math.round(gyro.getAngleY()) );
//        System.out.print(" Yaw angle: " + Math.round(gyro.getAngleZ()));
//        System.out.println(" Elapsed Time (seconds): " + elapsedTime);
//        boolean result = SmartDashboard.putNumber( "X angle", gyro.getAngleX());
//        
//        result = SmartDashboard.putNumber( "Y angle", gyro.getAngleY());
//        
//        result = SmartDashboard.putNumber("Z angle", gyro.getAngleZ());
//    	}
    }

    @Override
    public void autonomousInit() {
    	System.out.println("Autonomous has begun");
        autonomousCommand = autoProgramChoice.getSelected();
        System.out.println(startingPosition.getSelected().name());
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
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
    	System.out.println("teleop has begun)");
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        gyro.reset();
        gyro.calibrate();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        if (i % 50 == 0) {
            boolean result = SmartDashboard.putNumber( "X angle", gyro.getAngleX());
            
            result = SmartDashboard.putNumber( "Y angle", gyro.getAngleY());
            
            result = SmartDashboard.putNumber("Z angle", gyro.getAngleZ());
        	}
    }
}
