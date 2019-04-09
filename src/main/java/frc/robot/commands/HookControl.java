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
  boolean goingDown = false;
  boolean goingUp = false;
  long time = 0;
  @Override
  protected void execute() {
    
    // if(OI.logitech.getRawButton(5)){
    //   Robot.hook.setHookMotor(.108);
    // }else if(OI.logitech.getRawButton(6)){
    //   Robot.hook.setHookMotor(-.1392);
    // }else{
    //   Robot.hook.setHookMotor(0);
    // }
    
    if(OI.logitech.getRawButton(5)&&DownState){//Start Raising
      goingUp = true;
      Robot.hook.setHookMotor(.11*3);
      time = System.currentTimeMillis();//Gets current time
    }else if(OI.logitech.getRawButton(6)&&UpState){//Start Lowering
      goingDown=true;
      Robot.hook.setHookMotor(-.1392*2);
      time = System.currentTimeMillis();
    }
    boolean timeOut = System.currentTimeMillis()-time>2000;//Stop if it takes longer than 2 sec
    if(goingUp&&(RobotMap.upperSwitch.get()||OI.logitech.getBButton()||timeOut)){//Stop Raising
      Robot.hook.setHookMotor(0);
      DownState = false;
      UpState = true;
      goingUp = false;
    }else if(goingDown&&(RobotMap.lowerSwitch.get()||OI.logitech.getBButton()||timeOut)){//Stop Lowering
      Robot.hook.setHookMotor(0);
      DownState = true;
      UpState = false;
      goingDown = false;
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
