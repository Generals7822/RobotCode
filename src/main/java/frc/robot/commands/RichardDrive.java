/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;
import frc.robot.Main;
import frc.robot.OI;

public class RichardDrive extends Command {//Main Driving Command
  public RichardDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driving);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("initialized");
  }
  public static boolean slowMode = false;
  public static boolean superMode = false;
  public static boolean yPressed = false;
  public static boolean xPressed = false;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motor Intiaited
    SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
    leftmg.setInverted(true);//Flip left motor to get it going the right direction

    boolean yBtn = OI.logitech.getYButton();
    boolean xBtn = OI.logitech.getXButton();//Get states of X and Y button and save them
    
    //TO TEST: slow-mode for demos
    Robot.driving.RichardDrives(leftmg, rightmg, .1*OI.logitech.getRawAxis(1), -.1*OI.logitech.getRawAxis(4));
    /*
    if(yBtn&&!yPressed&&!slowMode){//set slow mode if yBtn is pressed and it is not in slow mode
      yPressed=true;
      slowMode=true;
    }
    if(yBtn&&!yPressed&&slowMode){//turn off slowMode if yButton is pressed, and it wasn't pressed on the prev
      slowMode=false;
      yPressed=true;
    }
    if(!yBtn){
      yPressed =false;
    }

    if(xBtn&&!xPressed&&!superMode){//set super mode if xBtn is pressed and it is not in super mode
      xPressed=true;
      superMode=true;
    }
    if(xBtn&&!xPressed&&superMode){//turn off superMode if xButton is pressed, and it wasn't pressed on the prev
      superMode=false;
      xPressed=true;
    }
    if(!xBtn){
      xPressed =false;
    }
    if(superMode){//Super Mode Command
      Robot.driving.RichardDrives(leftmg, rightmg, OI.logitech.getRawAxis(1), -OI.logitech.getRawAxis(4));
    }
    else if(slowMode){//More sensitive driving
      Robot.driving.RichardDrives(leftmg, rightmg, .2*OI.logitech.getRawAxis(1), -.2*OI.logitech.getRawAxis(4));
    }else{//Normal Driving
     Robot.driving.RichardDrives(leftmg, rightmg, .4*OI.logitech.getRawAxis(1), -.4*OI.logitech.getRawAxis(4));
    }*/
}

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