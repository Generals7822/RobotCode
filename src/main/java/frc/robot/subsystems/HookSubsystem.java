/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Victor;
import frc.robot.OI;

public class HookSubsystem extends Subsystem {
  public static Victor robotUpMotor = RobotMap.robotUpMotor;
  public static Victor hookUpMotor = RobotMap.hookUpMotor; 
  
  @Override
  public void initDefaultCommand() {

  }

  public void HookUp() 
  {
    if(OI.logitech.getStartButton()) //this raises the hook
    {
      robotUpMotor.set(0.6); //positive
    }
    
    if(OI.logitech.getBackButton()) //this lifts the robot up 
    {
      robotUpMotor.set(-0.6); //negative
    }

  }
}
