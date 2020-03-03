/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;//Info: auto format is Shift-Alt-F

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.commands.Drive_command;
import frc.robot.commands.HookCommand;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.commands.IntakeToShootCommand;
import frc.robot.commands.PixyAlignCommand;
import frc.robot.subsystems.IntakeToShootSubsystem;
import frc.robot.subsystems.PixyAlignSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.AutonomousSubsystem;
// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorMatch;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */

  //public final I2C.Port i2cPort= I2C.Port.kOnboard;
  public static PixyAlignSubsystem align = new PixyAlignSubsystem();
  public static DriveSubsystem driving = new DriveSubsystem();
  public static IntakeToShootSubsystem intake = new IntakeToShootSubsystem();
  public static HookSubsystem hook= new HookSubsystem(); 
  public static UsbCamera camera1;
  public static DigitalInput elevatorLimit= new DigitalInput(1);
  public static DigitalInput hookLimit= new DigitalInput(2);
  static long autoDriveTime; 

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  //boolean LEDOn= true;
  // private final ColorSensorV3 m_colorSensor= new ColorSensorV3(i2cPort);
  // private final ColorMatch m_colorMatcher= new ColorMatch();

  // private final Color kBlueTargetWithLED = ColorMatch.makeColor(0.173096, 0.446533, 0.380615);
  // private final Color kGreenTargetWithLED = ColorMatch.makeColor(0.214600,0.5195, 0.262);
  // private final Color kRedTargetWithLED = ColorMatch.makeColor(0.353516, 0.430176, 0.215820);
  // private final Color kYellowTargetWithLED = ColorMatch.makeColor(0.279297,.515137, 0.205566);
  
  // private final Color kBlueTargetWithoutLED = ColorMatch.makeColor(0.185, 0.473, 0.309);
  // private final Color kGreenTargetWithoutLED = ColorMatch.makeColor(0.2277,0.579, 0.1950);
  // private final Color kRedTargetWithoutLED = ColorMatch.makeColor(0.613, 0.314, 0.071);
  // private final Color kYellowTargetWithoutLED = ColorMatch.makeColor(0.411,.504, 0.080);
  
  // String colorLEDOff;
  // String colorLEDOn;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code
   */
  @Override
  public void robotInit() {
    //vision= new Vision();
    CameraServer.getInstance().startAutomaticCapture(0);
    
    //CameraServer.getInstance().startAutomaticCapture(1);
    // RobotMap.pixy.init();
    //SmartDashboard.putString("DB/String 0", "String");
    
    // m_colorMatcher.addColorMatch(kBlueTargetWithoutLED);
    // m_colorMatcher.addColorMatch(kGreenTargetWithoutLED);
    // m_colorMatcher.addColorMatch(kRedTargetWithoutLED);
    // m_colorMatcher.addColorMatch(kYellowTargetWithoutLED);
    
    // m_colorMatcher.addColorMatch(kBlueTargetWithLED);
    // m_colorMatcher.addColorMatch(kGreenTargetWithLED);
    // m_colorMatcher.addColorMatch(kRedTargetWithLED);
    // m_colorMatcher.addColorMatch(kYellowTargetWithLED);
  }

  /**
   * This funck tion is called every robot packet, no matter the mode. Use this
   * for items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //CameraServer.getInstance().startAutomaticCapture(0);
    //CameraServer.getInstance().startAutomaticCapture(1);
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    // Color detectedColor= m_colorSensor.getColor();
    // double IR= m_colorSensor.getIR();
    // ColorMatchResult match= m_colorMatcher.matchClosestColor(detectedColor);
    
    //   if (match.color == kGreenTargetWithLED) {
    //     colorLEDOn = "Green";
    //   }
    //    else if (match.color == kRedTargetWithLED) {
    //     colorLEDOn = "Red";
    //   } else if (match.color == kYellowTargetWithLED) {
    //     colorLEDOn = "Yellow";
    //   } 
    //   else if (match.color == kBlueTargetWithLED) {
    //     colorLEDOn = "Blue";
    //   }
    //   else {
    //     colorLEDOn = "Unknown";
    //   }
    
    // if (match.color == kGreenTargetWithoutLED) {
    //   colorLEDOff = "Green";
    // }
    //  else if (match.color == kRedTargetWithoutLED) {
    //   colorLEDOff = "Red";
    // } else if (match.color == kYellowTargetWithoutLED) {
    //   colorLEDOff = "Yellow";
    // } 
    // else if (match.color == kBlueTargetWithoutLED) {
    //   colorLEDOff = "Blue";
    // }
    // else {
    //   colorLEDOff = "Unknown";
    // }
  
     /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("IR", IR);
    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("Confidence", match.confidence);
    // SmartDashboard.putString("Detected Color (LEDOn):", colorLEDOn);
    // SmartDashboard.putString("Detected Color (LEDOff):", colorLEDOff);
    

        /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */

    //int proximity = m_colorSensor.getProximity();

    //SmartDashboard.putNumber("Proximity", proximity);

  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() 
  {
    Robot.align.setDefaultCommand(new PixyAlignCommand());
    autoDriveTime = System.currentTimeMillis();
  }

  /**+
   * 
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    Scheduler.getInstance().run();
    if(Math.subtractExact(System.currentTimeMillis(), autoDriveTime)>3000)
    {
      RobotMap.elevatorMotor.setSpeed(-0.6);
    }
    if(Math.subtractExact(System.currentTimeMillis(), autoDriveTime)>13500)
    {
      RobotMap.elevatorMotor.setSpeed(0);
      RobotMap.shootMotor.setSpeed(0);
      RobotMap.leftMotor1.setSpeed(0.25);
      RobotMap.leftMotor2.setSpeed(0.25);
      RobotMap.rightMotor1.setSpeed(-0.25);
      RobotMap.rightMotor2.setSpeed(-0.25);
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    Robot.driving.setDefaultCommand(new Drive_command());
    Robot.intake.setDefaultCommand(new IntakeToShootCommand());
    Robot.hook.setDefaultCommand(new HookCommand());
    //Robot.align.setDefaultCommand(new PixyAlignCommand());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    LiveWindow.setEnabled(true);
  }
}