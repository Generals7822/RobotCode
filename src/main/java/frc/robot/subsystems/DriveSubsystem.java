/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public static Victor lmotor1= RobotMap.leftMotor1;
public static Victor lmotor2= RobotMap.leftMotor2;
public static Victor rmotor1 = RobotMap.rightMotor1;
public static Victor rmotor2 = RobotMap.rightMotor2;

  public DriveSubsystem()
{

}

public void arcade(double left, double right)
{
  SpeedControllerGroup leftmg = new SpeedControllerGroup(lmotor1, lmotor2);
  SpeedControllerGroup rightmg = new SpeedControllerGroup(rmotor1, rmotor2);
  DifferentialDrive myDrive = new DifferentialDrive(leftmg, rightmg);
  myDrive.arcadeDrive(left, right);
}

public void squareSpeed(double left, double right)
{
  lmotor2.set(-left*Math.abs(left));
  lmotor1.set(-left*Math.abs(left));
  rmotor2.set(right*Math.abs(right));
  rmotor1.set(right*Math.abs(right));
}

public void speed(double left, double right)
{
  lmotor2.set(-left);
  lmotor1.set(-left);
  rmotor2.set(right);
  rmotor1.set(right);
}

public void squareSpeed(double speed)
{
  lmotor2.set(-speed*Math.abs(speed));
  lmotor1.set(-speed*Math.abs(speed));
  rmotor2.set(speed*Math.abs(speed));
  rmotor1.set(speed*Math.abs(speed));
}

public void speed(double speed)
{
  lmotor2.set(-speed);
  lmotor1.set(-speed);
  rmotor2.set(speed);
  rmotor1.set(speed);
}

public void stop()
{
  lmotor2.set(0);
  lmotor1.set(0);
  rmotor2.set(0);
  rmotor1.set(0);
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void RichardDrives(SpeedControllerGroup lmg, SpeedControllerGroup rmg, double power, double direction)
  {
    lmg.set(Math.max(-power+direction,-1)*Math.abs(Math.max(-power+direction,-1)));
    rmg.set(Math.min(-direction-power,1)*Math.abs(Math.min(-direction-power,1)));
    //non-Squared:
    //lmg.set(Math.max(-power+direction,-1));
    //rmg.set(Math.min(-direction-power,1));
    //Squared:
    lmg.set(Math.max(-power+direction,-1)*Math.abs(Math.max(-power+direction,-1)));
    rmg.set(Math.min(-direction-power,1)*Math.abs(Math.min(-direction-power,1)));
    /*
    //if direction is positive tank drive turn to the right
    if (direction > 0) {

      if (power < 0) {
        power = Math.abs(power);
        power = -1 * (Math.pow(power, 0.5));
        direction = Math.pow(direction, 0.5);

        rmg.set(power - direction);
        lmg.set(Math.min((power + direction), 1));
      }

      else {
        power = Math.pow(power, 0.5);
        direction = Math.pow(direction, 0.5);
        rmg.set(power - direction);
        lmg.set(Math.min((power + direction), 1));
      }

    }
    //if direction is negative tank drive turn to the left
    else if (direction <=0) {
      
      if (power < 0) {
        power = Math.abs(power);
        power = -1 * (Math.pow(power, 0.5));
      direction = Math.abs(direction);
      direction = -1 * (Math.pow(direction, 0.5));
      rmg.set(Math.max((power - direction), -1));
      lmg.set(power + direction);
      }
      else {
        power = Math.pow(power, 0.5);
        direction = Math.abs(direction);
        direction = -1 * (Math.pow(direction, 0.5));
        rmg.set(Math.max(power - direction, -1));
        lmg.set(power + direction);
      }
      
    }*/
  
  }
}
