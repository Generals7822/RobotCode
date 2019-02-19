/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

public class HookControl extends Command {
  public HookControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.hook);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }
  public static boolean UpState = true;
  public static boolean DownState = false;
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    // if(OI.logitech.getRawButton(5)){
    //   Robot.hook.setHookMotor(.108);
    // }else if(OI.logitech.getRawButton(6)){
    //   Robot.hook.setHookMotor(-.1392);
    // }else{
    //   Robot.hook.setHookMotor(0);
    // }

    if(OI.logitech.getRawButton(5)&&DownState){//Raising
      Robot.hook.setHookMotor(.11*3);
      while(!RobotMap.upperSwitch.get()&&!OI.logitech.getBButton()){
        try{Thread.sleep(2);}catch(Exception e){}
      }
      Robot.hook.setHookMotor(0);
      DownState = false;
      UpState = true;
    }else if(OI.logitech.getRawButton(6)&&UpState){//Lowering

      Robot.hook.setHookMotor(-.1392*2);
      while(!RobotMap.lowerSwitch.get()&&!OI.logitech.getBButton()){
        try{Thread.sleep(2);}catch(Exception e){}
      }
      Robot.hook.setHookMotor(0);
      DownState = true;
      UpState = false;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.hook.setHookMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
