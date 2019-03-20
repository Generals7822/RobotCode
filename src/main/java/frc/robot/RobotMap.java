/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.*;
import io.github.pseudoresonance.pixy2api.Pixy2;
import java.util.ArrayList;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  //public static int leftMotor = 1;
  public static Victor leftMotor1 = new Victor(0);
  public static Victor leftMotor2 = new Victor(1);
  public static Victor rightMotor1 = new Victor(2);
  public static Victor rightMotor2 = new Victor(3);
  public static Victor hookMotor = new Victor(4);
  public static Victor cargoMotor = new Victor(5);
  public static DigitalInput lowerSwitch = new DigitalInput(9);
  public static DigitalInput upperSwitch = new DigitalInput(8);
  public static Pixy2 pixy;


  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
