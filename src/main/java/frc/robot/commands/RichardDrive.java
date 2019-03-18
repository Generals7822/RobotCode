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

public class RichardDrive extends Command {
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

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);
    SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
    leftmg.setInverted(true);
    if(OI.logitech.getYButton()){//More sensitive driving
      Robot.driving.RichardDrives(leftmg, rightmg, .3*OI.logitech.getRawAxis(1), -.3*OI.logitech.getRawAxis(4));

    }else{
     Robot.driving.RichardDrives(leftmg, rightmg, .4*OI.logitech.getRawAxis(1), -.4*OI.logitech.getRawAxis(4));
    }
    
    //Robot.driving.RichardDrives(leftmg, rightmg, OI.leftJoy.getY(), OI.rightJoy.getX());
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