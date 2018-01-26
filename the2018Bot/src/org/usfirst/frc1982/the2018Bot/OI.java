package org.usfirst.frc1982.the2018Bot;

import org.usfirst.frc1982.the2018Bot.commands.AutonomousCommand;
import org.usfirst.frc1982.the2018Bot.commands.ClawToggle;
import org.usfirst.frc1982.the2018Bot.commands.ToggleGearShift;
import org.usfirst.frc1982.the2018Bot.commands.driveEnable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
    public Joystick xbox;


    public OI() {

        xbox = new Joystick(0);
        
        gearShifter = new JoystickButton(xbox, 2);
        gearShifter.whenPressed(new ToggleGearShift());
        
        clawToggler = new JoystickButton(xbox, 1);
        clawToggler.whenPressed(new ClawToggle());

    }

    public Joystick getxbox() {
        return xbox;
    }


}

