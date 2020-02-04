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
	public static boolean aPressed = false;
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
/*
	public static void initAuto() {
		FuncsToDo.clear();
		inputs.clear();
		lEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
		rEncoder = new Encoder(3, 4, false, Encoder.EncodingType.k2X);// SET the actual used ports
		lEncoder.setDistancePerPulse(1);// Change to actual value
		rEncoder.setDistancePerPulse(1);// Change to actual value

		leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);// Two Groups of Motor
																							// Intiaited
		rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
		leftmg.setInverted(true);// Flip left motor to get it going the right direction
		double robotAngle = toTerminalAngle(imu.getYaw());
		if (robotAngle > 180) {
			robotAngle = robotAngle - 360;
		}
		double absoluteHatchAngle;
		do {
			absoluteHatchAngle = getHatchAngle();// Comes from PixyCam
		} while (absoluteHatchAngle == Double.MAX_VALUE);// repeats until we get an actual value for Hatch location
		// Needs to be rethought, becuase it could loop for longer than periodic, and
		// crash

		double hatchAngle = absoluteHatchAngle + robotAngle;

		// The Robot currently calulates Distances and Movements relative to the Pixy2,
		// ideally this should be changed to the hook
		// One of doing this easily might be two Ultrasonics, or more advanced vision
		// processing

		boolean driveToRight = hatchAngle > 0;// Sees if we are driving to the left or right
		hatchAngle = Math.abs(hatchAngle);// changes everything to positive
		double ultDistance = ult.getRangeInches();// The distance from the ultrasonic to the cargo ship
		double PixytoUltDistance = 0;// Constant to set: distance between Pixy and Ultrasonic
		double angleFromPixytoUlt = 0;// Constant to set: angle from Pixy to Ultrasonic, between 0 and 90
		double distanceToCargoFromPixy = Math.sqrt(ultDistance * ultDistance + PixytoUltDistance * PixytoUltDistance
				- 2 * PixytoUltDistance * ultDistance * Math.cos((angleFromPixytoUlt + 90) * Math.PI / 180));
		double shipDistance = distanceToCargoFromPixy * Math.cos(robotAngle * Math.PI / 180);
		// Distance of Robot to Cargo Ship, from Pixy
		double hatchAngleRad = Math.PI * hatchAngle / 180;// Convert to Radians
		double inchesFromDrop = 5; // Constant to Set: how far in front the robot should be of the cargo ship when
									// lining up

		double pathDistance = Math.sqrt(shipDistance * shipDistance * Math.tan(hatchAngleRad) * Math.tan(hatchAngleRad)
				+ (shipDistance - inchesFromDrop) * (shipDistance - inchesFromDrop)); // Formula for calculating the
																						// distance to the spot in front
																						// of the cargo ship
		double pathTurnAngle = 180 / Math.PI
				* (hatchAngleRad
						+ Math.acos(-(-2 * shipDistance + inchesFromDrop * Math.cos(2 * hatchAngleRad) + inchesFromDrop)
								/ (2 * Math.cos(hatchAngleRad) * pathDistance)));// Formula for calculating angle to
																					// travel in

		// FuncsToDo.add(AutonomousCode::rotateTo);
		// if (driveToRight) {
		// 	// If we are going right, go right
		// 	inputs.add(pathTurnAngle);
		// } else {
		// 	inputs.add(360 - pathTurnAngle);
		// }
		// 	// If we are going left, go left
		// }
		
	// 	FuncsToDo.add(AutonomousCode::driveForward);
	// 	inputs.add(pathDistance);

	// 	FuncsToDo.add(AutonomousCode::rotateTo);// rotate towards the ship
	// 	inputs.add(0.0);

	// 	FuncsToDo.add(AutonomousCode::driveForward);
	// 	inputs.add(inchesFromDrop);

	// 	FuncsToDo.add(AutonomousCode::HatchDown);
	// 	inputs.add(0.0);

	// 	FuncsToDo.add(AutonomousCode::HatchUp);
	// 	inputs.add(0.0);
	// 	toDoInProgress = true;

	// }

	static int step = 0;
	static boolean toDoInProgress = false;

	public static void autonomous() {

		boolean aBtn = OI.logitech.getAButton();

		if (aBtn && !aPressed && !autoMode) {// set auto mode if aBtn is pressed and it is not in auto mode
			aPressed = true;
			autoMode = true;
			step = 0;// step counter for future additions to autonomous
		}
		if (aBtn && !aPressed && autoMode) {// turn off autoMode if aButton is pressed, and it wasn't pressed on the
											// prev
			autoMode = false;
			aPressed = true;
			toDoInProgress = false;

		}
		if (!aBtn) {
			aPressed = false;
		}

		if (autoMode) {
			testpixy();
			// if (toDoInProgress) {
			// 	ToDo(FuncsToDo, inputs);
			// } else {
			// 	//initAuto();
			// 	step++;
			// }
		}
	}

	static ArrayList<Consumer<Double>> FuncsToDo = new ArrayList<>();
	static ArrayList<Double> inputs = new ArrayList<>();

	static int currentTaskNum = 0;
	static boolean currentTaskDone = false;

	private static boolean ToDo(ArrayList<Consumer<Double>> funcs, ArrayList<Double> inputs) {
		funcs.get(currentTaskNum).accept(inputs.get(currentTaskNum));
		while (currentTaskDone) {
			currentTaskNum++;
			currentTaskDone = false;
			System.out.println(currentTaskNum);
			if (currentTaskNum == funcs.size()) {
				return true;
			}
			funcs.get(currentTaskNum).accept(inputs.get(currentTaskNum));
		}

		return false;
	}*/

	// public static void Ryan() {
	// 	PixyCamBlock[] blocks = George_Pixy.getBlocks();

	// 	if (blocks != null && blocks.length != 0) {
	// 		RobotMap.leftMotor1.set(-.25);
	// 		RobotMap.leftMotor2.set(-.25);
	// 		RobotMap.rightMotor1.set(.25);
	// 		RobotMap.rightMotor2.set(.25);
	// 	}
	// 	else{
	// 		RobotMap.leftMotor1.set(0);
	// 		RobotMap.leftMotor2.set(0);
	// 		RobotMap.rightMotor1.set(0);
	// 		RobotMap.rightMotor2.set(0);
	// 	}
	// }
	

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





