/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Drive_command;
import frc.robot.subsystems.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //public static Joystick leftJoy = new Joystick(1);
  // public static Joystick rightJoy = new Joystick(1);
  public static XboxController logitech = new XboxController(0);//Logitech Controller on port 0
  public static Joystick joystick = new Joystick(1);
  
  // Button button5l = new JoystickButton(leftJoy, 5);
  // Button button1l = new JoystickButton(leftJoy, 1);
  // Button button1r = new JoystickButton(rightJoy, 1);
  // Button button5r = new JoystickButton(rightJoy, 5);
  // Button button12r = new JoystickButton(rightJoy, 12);
  // Button button11r = new JoystickButton(rightJoy, 11);
  JoystickButton button5l= new JoystickButton(joystick, 1);
  

  public OI() {
    //button5l.whenPressed(new Drive_command());
    //button1l.whenPressed(new RichardDrive());
    //button1r.whenPressed(new Stop());
    //button5r.whenPressed(new MoreSensitiveDrive());
    // button12r.whileHeld(new hookUp());
    // button11r.whileHeld(new hookDown());

  }




  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
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

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
