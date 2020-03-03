/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.*;
import frc.robot.subsystems.*;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.

//this is where the motor objects are assigned to motor controller ports on the PWM

  public static Victor leftMotor1 = new Victor(0);//Motors on their respectives ports
  public static Victor leftMotor2 = new Victor(2);
  public static Victor rightMotor1 = new Victor(1);
  public static Victor rightMotor2 = new Victor(3);
  public static Victor shootMotor = new Victor(7);
  public static Victor intakeMotor= new Victor(8);
  public static Victor elevatorMotor = new Victor(9);
  public static Victor hookUpMotor = new Victor(6);
  public static Victor robotUpMotor = new Victor(4);
  
  public static George_Pixy pixy = new George_Pixy();//Pixy Cam
  
}

