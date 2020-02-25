/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


//import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.*;
import frc.robot.commands.AutonomousDrive;
import edu.wpi.first.wpilibj.drive.*;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

//import java.util.ArrayList;
//import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * Add your docs here.
 */
public class AutonomousSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public static Victor lmotor1= RobotMap.leftMotor1; //reset
public static Victor lmotor2= RobotMap.leftMotor2;
public static Victor rmotor1 = RobotMap.rightMotor1;
public static Victor rmotor2 = RobotMap.rightMotor2;

  public AutonomousSubsystem()
{

}

public void arcade(double left, double right)
{
  SpeedControllerGroup leftmg = new SpeedControllerGroup(lmotor1, lmotor2);
  SpeedControllerGroup rightmg = new SpeedControllerGroup(rmotor1, rmotor2);
  DifferentialDrive myDrive = new DifferentialDrive(leftmg, rightmg);
  myDrive.arcadeDrive(left, right);
}

public void speed(double left, double right)
{
  lmotor2.set(-left);
  lmotor1.set(-left);
  rmotor2.set(right);
  rmotor1.set(right);
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
    //This is a comment
    setDefaultCommand(new AutonomousDrive());
  }
  public void LeftMotorStrength(SpeedController lmg, SpeedController rmg)
  {
    lmg.set(0);
    rmg.set(0.8);
  }
  public void Drives(SpeedControllerGroup lmg, SpeedControllerGroup rmg, double power, double direction)
  {//Main Drive command
    //Squared:
    //lmg.set(Math.max(-power+direction,-1)*Math.abs(Math.max(-power+direction,-1))); 
    //rmg.set(Math.min(-direction-power,1)*Math.abs(Math.min(-direction-power,1)));
    //non-Squared:
    lmg.set(Math.max(-power+direction,-1));
    rmg.set(Math.min(-direction-power,1));//Sets the controls with arcade mode calculation
  
  }
  public void AutonomousStart(SpeedControllerGroup lmg, SpeedControllerGroup rmg)
  {
    lmg.set(-0.1);
    rmg.set(-0.1);
  }
  public void TimedDistance(SpeedControllerGroup lmg, SpeedControllerGroup rmg, Timer t)
  {
		Timer timer= t; //timer works in microseconds
		if(timer.get()<5)
		{
		lmg.set(0.25);
		rmg.set(0.25);
    }
    else
    {
		lmg.set(0);
		rmg.set(0);
    }
  }
}


