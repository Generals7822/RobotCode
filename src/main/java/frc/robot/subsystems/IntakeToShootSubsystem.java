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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.HookSubsystem;

public class IntakeToShootSubsystem extends Subsystem {

public static Victor intake1 = RobotMap.intakeMotor;
public static Victor elevator1= RobotMap.elevatorMotor;
public static Victor shooter1= RobotMap.shootMotor;

public static Timer elevatorTimer= new Timer();


  @Override
  public void initDefaultCommand() {

  }

  public void intakeToShoot()
  {
    if(OI.logitech.getXButton())
    {
        intake1.setSpeed(-0.6);
        elevator1.setSpeed(0);
        shooter1.setSpeed(0);
    }
    
    if(OI.logitech.getYButton())
      {
        intake1.setSpeed(0);
        elevator1.setSpeed(-0.6);
      }

    if(OI.logitech.getAButton())
    {      
      intake1.setSpeed(0);
      elevator1.setSpeed(0);
      shooter1.setSpeed(-0.75);
    }
    if(OI.logitech.getBButton())
    {
      intake1.setSpeed(0);
      elevator1.setSpeed(0);
      shooter1.setSpeed(0);
      HookSubsystem.hookUpMotor.set(0);
      HookSubsystem.robotUpMotor.set(0);
    }
    if(OI.logitech.getBumper(Hand.kLeft))
    {
      elevator1.setSpeed(0.6);
    }
      
  }
}
