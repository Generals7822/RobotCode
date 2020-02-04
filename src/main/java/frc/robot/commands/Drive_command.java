/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class Drive_command extends Command {//Main Driving Command
  public Drive_command() {
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
  public static boolean leftTestMode= false;
  public static boolean triggerPressed = false;
  public static boolean pixyMode= false;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //boolean liftingMotor= false;

    SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motor Intiaited
    SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
    leftmg.setInverted(true);//Flip left motor to get it going the right direction

    
    
    
    boolean yBtn = OI.logitech.getYButton();
    boolean xBtn = OI.logitech.getXButton();//Get states of X and Y button and save them
    boolean trigger= OI.joystick.getTriggerPressed();
    
    //slow-mode for demos
    //Robot.driving.RichardDrives(leftmg, rightmg, .25*OI.logitech.getRawAxis(1), -.25*OI.logitech.getRawAxis(4));
    
    // if(yBtn&&!yPressed&&!slowMode){//set slow mode if yBtn is pressed and it is not in slow mode
    //   yPressed=true;
    //   slowMode=true;
    // }
    // if(yBtn&&!yPressed&&slowMode){//turn off slowMode if yButton is pressed, and it wasn't pressed on the prev
    //   slowMode=false;
    //   yPressed=true;
    // }
    // if(!yBtn){
    //   yPressed =false;
    // }

    // if(xBtn&&!xPressed&&!superMode){//set super mode if xBtn is pressed and it is not in super mode
    //   xPressed=true;
    //   superMode=true;
    // }
    // if(xBtn&&!xPressed&&superMode){//turn off superMode if xButton is pressed, and it wasn't pressed on the prev
    //   superMode=false;
    //   xPressed=true;
    // }
    // if(!xBtn){
    //   xPressed =false;
    // }
    // if(trigger&&!leftTestMode)
    // {
    //   triggerPressed= true;
    //   leftTestMode= true;
    // }
    if(yBtn)
    {
      pixyMode=true;
    }
    if(superMode){//Super Mode Command
      Robot.driving.RichardDrives(leftmg, rightmg, OI.logitech.getRawAxis(1), -OI.logitech.getRawAxis(4));
    }
    else if(slowMode){//More sensitive driving
      Robot.driving.RichardDrives(leftmg, rightmg, .2*OI.logitech.getRawAxis(1), -.2*OI.logitech.getRawAxis(4));
    }
    else if(leftTestMode)
    {
      Robot.driving.RichardDrives(leftmg, rightmg, 0, -.8*OI.joystick.getRawAxis(1));
    }
    else if(pixyMode)
    {
      Robot.pixy.setDistance();
    }
    else{//Normal Driving
     Robot.driving.itwasMicah(leftmg, rightmg);
    }
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