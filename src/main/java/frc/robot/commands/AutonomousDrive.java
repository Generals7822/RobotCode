/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDrive extends Command {//Main Driving Command
  public AutonomousDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.auto); //Robot.auto is declared in the Robot class and is an autonomous subsystem
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("initialized");
    
  }
  public static boolean LeftStart=false;
  public static boolean MiddleStart=false;
  public static boolean RightStart=false;
  public static boolean autoMode=false;
  public static boolean yBtn=false;
  public static boolean pixyMode=false;
  public static Timer timer= new Timer();
  

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //boolean liftingMotor= false;

    SpeedControllerGroup leftmg = new SpeedControllerGroup(AutonomousSubsystem.lmotor1, AutonomousSubsystem.lmotor2);//Two Groups of Motor Intiaited
    SpeedControllerGroup rightmg = new SpeedControllerGroup(AutonomousSubsystem.rmotor1, AutonomousSubsystem.rmotor2);
    leftmg.setInverted(true);//Flip left motor to get it going the right direction

  
    
    //slow-mode for demos
    //Robot.driving.RichardDrives(leftmg, rightmg, .25*OI.logitech.getRawAxis(1), -.25*OI.logitech.getRawAxis(4));
    
    // if(LeftStart){
    //   Robot.driving.RichardDrives(leftmg, rightmg, OI.logitech.getRawAxis(1), -OI.logitech.getRawAxis(4));
    // }
    //else if(MiddleStart){
      //Robot.auto.AutonomousStart(leftmg, rightmg);
      //timer.start();
      if(yBtn)
      {
        pixyMode=true;
      }
      Robot.auto.TimedDistance(leftmg, rightmg, Robot.autoTimer);
     // Robot.auto.TimedDistance(leftmg, rightmg);
    //}
    // else if(RightStart)
    // {
    //   Robot.driving.RichardDrives(leftmg, rightmg, 0, -.8*OI.joystick.getRawAxis(1));
    // }
    // else{//Normal Driving
    //  Robot.driving.LeftMotorStrength(leftmg, rightmg);
    // }
  }
  //Robot.driving.RichardDrives(leftmg, rightmg, 1, -1);

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driving.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}