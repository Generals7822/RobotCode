/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.lang.Object;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.*;
//import com.analog.adis16448.frc.*;
import java.util.ArrayList;
import java.util.function.Consumer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
/**
 * All ToDo Functions must take in a double and return void, all changes should
 * be made through field variables This is a first attempt at Auto Code, all of
 * this is untested and needs testing /bug fixes in order to work Requires:
 * Gyro, Encoder, Pixy, and Ultrasonic Program computes distance and angle based
 * on the Pixy
 * 
 */

public class AutonomousCode_tinkering {
	//public static boolean aPressed = false;
	public static boolean autoMode = false;
	//public static ADIS16448_IMU imu = new ADIS16448_IMU();
	//public static Ultrasonic ult = new Ultrasonic(1, 1);// Set Actual Input and Output values
	static SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motors Intiaited
    static SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
	//static Encoder rEncoder;
	//static Encoder lEncoder;
	public AutonomousCode_tinkering()
	{
		autoMode=true;
	}

	

	public static void testpixy() {
		PixyCamBlock[] blocks = George_Pixy.getBlocks();
		
		double addRight;
		double addLeft;
		if (blocks != null && blocks.length != 0) {
			double currentDeg = toTerminalAngle(160 - blocks[0].xCenter);
			if (currentDeg < 5 || currentDeg > 355) {// If it is within 5 degrees, stop
				addRight = 0;
				addLeft = 0;
				// currentTaskDone = true;
				// return;
			}
			if (currentDeg > 180) {
				double degRot = 360 - currentDeg;
				// System.out.println("Rotate " + degRot + " in CCW dir.");
				addLeft = -Math.max(-degRot / 500, -0.5);
				addRight = Math.min(degRot / 500, 0.5);
			} else {
				double degRot = currentDeg;
				// System.out.println("Rotate " + degRot + " in CW dir.");
				addLeft = -Math.min(degRot / 500, 0.5);
				addRight = Math.max(-degRot / 500, -0.5);
			}

			if (blocks[0].height <= 45) {
				RobotMap.leftMotor1.set(-.25 + addLeft);
				RobotMap.leftMotor2.set(-.25 + addLeft);
				RobotMap.rightMotor1.set(.25 + addRight);
				RobotMap.rightMotor2.set(.25 + addRight);
			} else if (blocks[0].height >= 55) {
				RobotMap.leftMotor1.set(.25 + addLeft);
				RobotMap.leftMotor2.set(.25 + addLeft);
				RobotMap.rightMotor1.set(-.25 + addRight);
				RobotMap.rightMotor2.set(-.25 + addRight);
			} else {
				RobotMap.leftMotor1.set(0);
				RobotMap.leftMotor2.set(0);
				RobotMap.rightMotor1.set(0);
				RobotMap.rightMotor2.set(0);
			}

		} else {
			RobotMap.leftMotor1.set(0);
			RobotMap.leftMotor2.set(0);
			RobotMap.rightMotor1.set(0);
			RobotMap.rightMotor2.set(0);
		}
	}
	

	private static void rotateBy(double targetDeg) 
	{
//First approach to setting direction, should defenitely be
												// improved after testing
												
		double currentDeg = toTerminalAngle(targetDeg);
		// SmarthDasboard.putNumber("Yaw", imu.getYaw());
		// SmartDashboard.putNumber("XAngle", imu.getAngleX());
		// SmartDashboard.putNumber("YAngle", imu.getAngleY());
		// SmartDashboard.putNumber("ZAngle", imu.getAngleZ());

		if (currentDeg < 5 || currentDeg > 355) {// If it is within 5 degrees, stop
			leftmg.set(0);
			rightmg.set(0);
			// currentTaskDone = true;
			// return;
		}
		if (currentDeg > 180) {
			double degRot = 360 - currentDeg;
			// System.out.println("Rotate " + degRot + " in CCW dir.");
			leftmg.set(-Math.max(-degRot / 500, -0.5));
			rightmg.set(Math.min(degRot / 500, 0.5));
		} else {
			double degRot = currentDeg;
			// System.out.println("Rotate " + degRot + " in CW dir.");
			leftmg.set(-Math.min(degRot / 500, 0.5));
			rightmg.set(Math.max(-degRot / 500, -0.5));
		}
		
	}
	static boolean sAPressed = false;
	static boolean sSlowmode = false;

	// private static void sarahs_tinker() {
	// 	boolean abtn = OI.logitech.getAButton();
	// 	if (abtn && !sAPressed) {
	// 		sAPressed = true;
	// 		sSlowmode = true;
	// 	}
	// 	if (abtn && sAPressed){
	// 		sAPressed = false;
	// 		sSlowmode =false; 
	// 	}
	// }
	
	private static void rotateDrive(double targetDeg) 
	{
//First approach to setting direction, should defenitely be
												// improved after testing
												
		double currentDeg = toTerminalAngle(targetDeg);
		// SmarthDasboard.putNumber("Yaw", imu.getYaw());
		// SmartDashboard.putNumber("XAngle", imu.getAngleX());
		// SmartDashboard.putNumber("YAngle", imu.getAngleY());
		// SmartDashboard.putNumber("ZAngle", imu.getAngleZ());

		if (currentDeg < 5 || currentDeg > 355) {// If it is within 5 degrees, stop
			leftmg.set(0);
			rightmg.set(0);
			// currentTaskDone = true;
			// return;
		}
		if (currentDeg > 180) {
			double degRot = 360 - currentDeg;
			// System.out.println("Rotate " + degRot + " in CCW dir.");
			leftmg.set(-Math.max(-degRot / 500, -0.5));
			rightmg.set(Math.min(degRot / 500, 0.5));
		} else {
			double degRot = currentDeg;
			// System.out.println("Rotate " + degRot + " in CW dir.");
			leftmg.set(-Math.min(degRot / 500, 0.5));
			rightmg.set(Math.max(-degRot / 500, -0.5));
		}
		
	}
	
//public static double getHatchAngle() {}

		// Complete w/ PixyCam
		// Test Pixy Code

		// if (!SmartDashboard.getString("DB/String 2", "null").equals("Running")) {
		// SmartDashboard.putString("DB/String 2", "Running");

		// }

	// 	ArrayList<Block> blocks = RobotMap.pixy.getCCC().getBlocks();
	// 	if (!SmartDashboard.getString("DB/String 1", "null").equals("" +
	// 	blocks.size())) {
	// 	SmartDashboard.putString("DB/String 1", "" + blocks.size());

	// 	}
	// 	if (blocks.size() >= 2) {// Intial version, stands to be improved
	// 		Block leftBlock;
	// 		Block rightBlock;
	// 		if (blocks.get(0).getX() < blocks.get(1).getX()) {
	// 		leftBlock = blocks.get(0);
	// 		rightBlock = blocks.get(1);
	// 		} else {
	// 		leftBlock = blocks.get(1);
	// 		rightBlock = blocks.get(0);
	// 		}
	// 		return (30 / 158 * (blocks.get(0).getX() + blocks.get(1).getX()) / 2 - 158);
	// 		//Uses Pixy2 FOV of 30 deg and resolution of 316 to calculate approx Angle
	// 	}
	// 	return Double.MAX_VALUE;// return Max Value if there is not two blocks

	// }

	private static double toTerminalAngle(double angle) {// Converts to Terminal Angle
		return (angle % 360 + 360) % 360;
	}}
	

	//private static void rotateTo(double targetDeg) 
	//{}
//}// First approach to setting direction, should defenitely be
												// improved after testing
		/*										
		//\\double currentDeg = toTerminalAngle(targetDeg - imu.getYaw());
		SmarthDasboard.putNumber("Yaw", imu.getYaw());
		SmartDashboard.putNumber("XAngle", imu.getAngleX());
		SmartDashboard.putNumber("YAngle", imu.getAngleY());
		SmartDashboard.putNumber("ZAngle", imu.getAngleZ());

		if (currentDeg < 2 || currentDeg > 310) {// If it is within 2 degrees, stop
			leftmg.set(0);
			rightmg.set(0);
			currentTaskDone = true;
			return;
		}
		if (currentDeg > 0) {
			double degRot = 360 - currentDeg;
			// System.out.println("Rotate " + degRot + " in CCW dir.");
			leftmg.set(Math.max(-degRot / 100, -1));
			rightmg.set(Math.min(degRot / 100, 1));
		} else {
			double degRot = currentDeg;
			// System.out.println("Rotate " + degRot + " in CW dir.");
			leftmg.set(Math.min(degRot / 100, 1));
			rightmg.set(Math.max(-degRot / 100, -1));
		}
		
	}
/*
	static boolean initDrive = true;

	private static void driveForward(double inches) {// drives forwards the set amount
		// System.out.println("Driving Forward " + inches + " inches.");
		if (initDrive) {// Reset the encoders at the intial starting point
			lEncoder.reset();
			rEncoder.reset();
		}
		initDrive = false;// finish intial set-up
		double approxDistance = (lEncoder.getDistance() + rEncoder.getDistance()) / 2;
		// estimate distance w/ the average of the encoders

		if (inches - approxDistance < .5 && inches - approxDistance > -.5) {
			currentTaskDone = true;// Within half an inch, declare done and set initDrive for next drive command
			initDrive = true;
			return;
		}
		leftmg.set(Math.min(1, (inches - approxDistance) / 12));// Set the motors proportional to distance
		rightmg.set(Math.min(1, (inches - approxDistance) / 12));

	}
}
}
*/





