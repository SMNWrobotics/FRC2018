
package org.usfirst.frc1982.the2018Bot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    
    public static Compressor pneumaticsCompressor;
    public static Solenoid pneumaticsGearShifter;
    
    //claw solenoids:
    public static DoubleSolenoid pneumaticsClaw;
    public static Solenoid pneumaticsPunch;
    public static DoubleSolenoid pneumaticsHinge;
    
    public static WPI_TalonSRX driveRight;
    public static WPI_TalonSRX driveLeft;
    public static WPI_VictorSPX driveRightSlaveOne;
    public static WPI_VictorSPX driveRightSlaveTwo;
    public static WPI_VictorSPX driveLeftSlaveOne;
    public static WPI_VictorSPX driveLeftSlaveTwo;
    
    
    public static WPI_TalonSRX elevatorLeft;
    public static WPI_TalonSRX elevatorRight;
    public static void init() {
        
        pneumaticsCompressor = new Compressor(0);
        
        pneumaticsGearShifter = new Solenoid(0, 0);
        pneumaticsClaw = new DoubleSolenoid(0,2,3);
        //open is: kForward
        //closed is: kReverse
        pneumaticsPunch = new Solenoid(0, 5);
        //out is: 
        //in is: 
        pneumaticsHinge = new DoubleSolenoid(0,6,7);
        //up is:
        //down is:
        
        //drive talonSRXs
        driveRight = new WPI_TalonSRX(1);
        
        driveRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
        driveRight.setSelectedSensorPosition(0,0,0);
        driveRight.setSensorPhase(false);
        
        driveLeft = new WPI_TalonSRX(0);
        
        driveLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
        driveLeft.setSelectedSensorPosition(0,0,0);
        driveLeft.setSensorPhase(false);
        
        driveLeft.setInverted(true);
        driveRight.setInverted(false);
        
        driveRight.setSafetyEnabled(false);
        driveLeft.setSafetyEnabled(false);
        
        driveRight.configSetParameter(ParamEnum.ePeakCurrentLimitAmps, 100, 0, 0, 20);
        driveLeft.configSetParameter(ParamEnum.ePeakCurrentLimitAmps, 100, 0, 0, 20);
        
        //slave VictorSPXs:
        //odds on right side:
        driveRightSlaveOne = new WPI_VictorSPX(3);
        driveRightSlaveTwo = new WPI_VictorSPX(5);
        driveRightSlaveOne.setInverted(false);
        driveRightSlaveOne.follow(driveRight);
        driveRightSlaveTwo.follow(driveRight);
        
        //evens on the left:
        driveLeftSlaveOne = new WPI_VictorSPX(2);
        driveLeftSlaveTwo = new WPI_VictorSPX(4);
        driveLeftSlaveOne.setInverted(true);
        driveLeftSlaveOne.follow(driveLeft);
        driveLeftSlaveTwo.setInverted(true);
        driveLeftSlaveTwo.follow(driveLeft);
        
        ///////elevator TalonSRXs:
        elevatorLeft = new WPI_TalonSRX(3); //////TODO: change numbers for these!!!
        
        elevatorRight = new WPI_TalonSRX(4);
        
        elevatorRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,20);
        elevatorRight.setSensorPhase(false);
        elevatorRight.setSelectedSensorPosition(0, 0, 0);
        
        elevatorRight.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        elevatorRight.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0, 0);
        
        elevatorRight.config_kP(0, 0.1, 0);
        elevatorRight.config_kI(0, 0, 0);
        elevatorRight.config_kD(0, 0, 0);
        elevatorRight.config_kF(0, 0, 0);
        
        elevatorRight.configForwardSoftLimitEnable(true, 0);
        elevatorRight.configForwardSoftLimitThreshold(27500, 0);
        
        System.out.println(elevatorLeft.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, 4, 0));
        elevatorLeft.follow(elevatorRight);
    }
}