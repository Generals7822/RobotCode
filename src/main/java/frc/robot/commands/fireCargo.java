/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class fireCargo extends Command {
  private double m_timeout = 0.1;
  public fireCargo() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cargo);
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(0.1);
    Robot.cargo.fireCargoMotor(.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   // if (OI.logitech.getRawButton(1))
     // Robot.cargo.fireCargoMotor(.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.cargo.fireCargoMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.cargo.fireCargoMotor(0);
  }
}
