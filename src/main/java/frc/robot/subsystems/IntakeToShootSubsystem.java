/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class IntakeToShootSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

public static Victor intake1 = RobotMap.intakeMotor;
public static Victor elevator1= RobotMap.elevatorMotor;
public static Victor shooter1= RobotMap.shootMotor;
Timer elevatorTimer= new Timer();

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void intakeToShoot(Victor v)
  {
    intake1=v;
    if(OI.logitech.getYButtonPressed())
    {
      intake1.setSpeed(-0.5);
      elevator1.setSpeed(0);
      shooter1.setSpeed(0);
    }
    if(OI.logitech.getXButton())
      {
        elevatorTimer.start();
        intake1.setSpeed(0);
        elevator1.setSpeed(-0.5);
        shooter1.setSpeed(0);
        if(elevatorTimer.get()>2)
        {
          elevator1.setSpeed(0);
          elevatorTimer.reset();
        }
      }
      
    if(OI.logitech.getAButton())
    {
      intake1.setSpeed(0);
      elevator1.setSpeed(0);
      shooter1.setSpeed(-0.5);
    }
    if(OI.logitech.getBButton())
    {
      intake1.setSpeed(0);
      elevator1.setSpeed(0);
      shooter1.setSpeed(0);
    }   
      
  }
  public void elevator(Victor v)
  {
    elevator1=v;
    elevator1.setSpeed(0.25);
  }
}
