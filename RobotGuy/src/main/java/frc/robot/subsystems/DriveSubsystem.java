
package frc.robot.subsystems;


//imports 
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import main.java.frc.robot.commands.DriveManuallyCommand;

/**
 * This is the subsystem for all the drivetrain methods:
 * 
 * History of Edits: 
 * 
 * 1/31/19: 
 */


public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  //instantiating motor control objects, ports are stored in robotmap as constants 
  public WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(RobotMap.leftMotorPort_alpha);
  public WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(RobotMap.leftMotorPort_beta);
  public WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(RobotMap.rightMotorPort_alpha);
  public WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(RobotMap.rightMotorPort_beta);

  //linking speed controllers 
  SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(leftMotor1, leftMotor2);
  SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(rightMotor1, rightMotor2);


  //instantiate differentiable drive object and pass motor controllers 
  public DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);


  //constructor 
  public DriveSubsystem() {
    //basically the init method of this subsytem if u want to configure motors at all do it here
   
  }


  //add manualDrive() method 
  public void manualDrive(double ls, double rs) {
    
    double leftSpeed = ls; 
    double rightSpeed = rs; 
    drive.tankDrive(leftSpeed, rightSpeed);

  }

  //method of driving for more sensetive movements 
  public void sensitiveDrive(double ls, double rs) {
    
    double leftSpeed = ls; 
    double rightSpeed = rs; 

    if (rightSpeed > 0.25 ) 
      rightSpeed = 0.25; 
    else if (leftSpeed > 0.25)
      leftSpeed = 0.25; 
    else if (rightSpeed < -0.25)
      rightSpeed = -0.25; 
    else if (leftSpeed < -0.25)
      leftSpeed = -0.25; 

    drive.tankDrive(leftSpeed, rightSpeed);
  }





  @Override
  public void initDefaultCommand() {
    // setting deafault command for tank drive.
    setDefaultCommand(new DriveManuallyCommand());
  }
}
