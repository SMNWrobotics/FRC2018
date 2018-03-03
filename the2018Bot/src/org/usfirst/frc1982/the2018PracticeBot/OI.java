package org.usfirst.frc1982.the2018Bot;

import org.usfirst.frc1982.the2018Bot.commands.ClawToggle;
import org.usfirst.frc1982.the2018Bot.commands.ElevatorManual;
import org.usfirst.frc1982.the2018Bot.commands.ElevatorScaleHeight;
import org.usfirst.frc1982.the2018Bot.commands.ElevatorSetpointManual;
import org.usfirst.frc1982.the2018Bot.commands.ElevatorSetpoint;
import org.usfirst.frc1982.the2018Bot.commands.ElevatorSwitchHeight;
import org.usfirst.frc1982.the2018Bot.commands.ToggleGearShift;
import org.usfirst.frc1982.the2018Bot.commands.ToggleHinge;
import org.usfirst.frc1982.the2018Bot.commands.TogglePushPiston;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    public JoystickButton gearShifter;
    public JoystickButton clawToggler;
    public JoystickButton elevatorManual;
    public JoystickButton elevatorSetPointManual;
    public JoystickButton elevatorSetPoint;
    public JoystickButton elevatorSetSwitch;
    public JoystickButton elevatorSetScale;
    public JoystickButton hingeToggler;
    public JoystickButton punchToggler;
    public Joystick driver;
    public Joystick opBox;


    public OI() {
    	
    	opBox = new Joystick(1);
        driver = new Joystick(0);
        
        gearShifter = new JoystickButton(driver, 2);
        gearShifter.whenPressed(new ToggleGearShift());
        
        clawToggler = new JoystickButton(driver, 1);
        clawToggler.whenPressed(new ClawToggle());
        
        elevatorManual = new JoystickButton(driver,12);
        elevatorManual.whenPressed(new ElevatorManual());
        
        elevatorSetPointManual = new JoystickButton(driver, 5);
        elevatorSetPointManual.whenPressed(new ElevatorSetpointManual());
        
        elevatorSetPoint = new JoystickButton( driver, 6);
        elevatorSetPoint.whenPressed(new ElevatorSetpoint());
        
        elevatorSetSwitch = new JoystickButton(driver, 7);
        elevatorSetSwitch.whenPressed(new ElevatorSwitchHeight());
        
        elevatorSetScale = new JoystickButton(driver, 8);
        elevatorSetScale.whenPressed(new ElevatorScaleHeight());
        
        hingeToggler = new JoystickButton(driver, 9);
        hingeToggler.whenPressed(new ToggleHinge());
        
        punchToggler = new JoystickButton(driver, 4);
        punchToggler.whenPressed(new TogglePushPiston());
    }
    
}

