/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.*;
import com.analog.adis16448.frc.*;
import java.util.ArrayList;
import java.util.function.Consumer;
import frc.robot.subsystems.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

public class AutonomousCode { // All ToDo Functions must take in a double and be void, all changes should be
								// made through field variables
	// This is a first attempt at Auto Code, all of this is untested and will almost
	// certainly not work
	// Requires: Gyro, Encoder, Pixy, and Ultrasonic
	public static boolean aPressed = false;
	public static boolean autoMode = false;
	public static ADIS16448_IMU imu = new ADIS16448_IMU();
	public static Ultrasonic ult = new Ultrasonic(1, 1);// Set Actual Input and Output values
	static SpeedControllerGroup leftmg;
	static SpeedControllerGroup rightmg;
	static Encoder rEncoder;
	static Encoder lEncoder;

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

		double hatchAngle = robotAngle + getHatchAngle();// Comes from PixyCam

		boolean driveToRight = hatchAngle > 0;// Sees if we are driving to the left or right
		hatchAngle = Math.abs(hatchAngle);// changes everything to positive
		double shipDistance = ult.getRangeInches() * Math.cos(robotAngle * Math.PI / 180); // distance of Robot to Cargo
																							// Ship
		double hatchAngleRad = Math.PI * hatchAngle / 180;// Convert to Radians
		double inchesFromDrop = 6; // how far in front the robot should be of the cargo ship when lining up

		double pathDistance = Math.sqrt(shipDistance * shipDistance * Math.tan(hatchAngleRad) * Math.tan(hatchAngleRad)
				+ (shipDistance - inchesFromDrop) * (shipDistance - inchesFromDrop)); // Formula for calculating the
																						// distance to the spot in front
																						// of the cargo ship
		double pathTurnAngle = 180 / Math.PI
				* (hatchAngleRad
						+ Math.acos(-(-2 * shipDistance + inchesFromDrop * Math.cos(2 * hatchAngleRad) + inchesFromDrop)
								/ (2 * Math.cos(hatchAngleRad) * pathDistance)));// Formula for calculating angle to
																					// travel in

		FuncsToDo.add(AutonomousCode::rotateTo);
		if (driveToRight) {
			// If we are going right, go right
			inputs.add(pathTurnAngle);
		} else {
			inputs.add(360 - pathTurnAngle);// If we are going left, go left
		}
		FuncsToDo.add(AutonomousCode::driveForward);
		inputs.add(pathDistance);

		FuncsToDo.add(AutonomousCode::rotateTo);// rotate towards the ship
		inputs.add(0.0);

		FuncsToDo.add(AutonomousCode::driveForward);
		inputs.add(inchesFromDrop);

		FuncsToDo.add(AutonomousCode::HatchDown);
		inputs.add(0.0);

		FuncsToDo.add(AutonomousCode::HatchUp);
		inputs.add(0.0);
		toDoInProgress = true;

	}

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
			if (toDoInProgress) {
				ToDo(FuncsToDo, inputs);
			} else {
				initAuto();
				step++;
			}
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
	}

	public static double getHatchAngle() {

		// Complete w/ PixyCam
		// Test Pixy Code

		if (!SmartDashboard.getString("DB/String 2", "null").equals("Running")) {
			SmartDashboard.putString("DB/String 2", "Running");

		}

		ArrayList<Block> blocks = RobotMap.pixy.getCCC().getBlocks();
		if (!SmartDashboard.getString("DB/String 1", "null").equals("" + blocks.size())) {
			SmartDashboard.putString("DB/String 1", "" + blocks.size());

		}
		if (blocks.size() >= 2) {//Intial version, stands to be improved
			Block leftBlock;
			Block rightBlock;
			if (blocks.get(0).getX() < blocks.get(1).getX()) {
				leftBlock = blocks.get(0);
				rightBlock = blocks.get(1);
			} else {
				leftBlock = blocks.get(1);
				rightBlock = blocks.get(0);
			}
			return (30/158*(leftBlock.getX()+leftBlock.getX())/2-158);//Uses Pixy2 FOV of 30 deg and resolution of 316 to calculate approx Angle
			
		}
		return Double.MAX_VALUE;

	}

	private static double toTerminalAngle(double angle) {
		return (angle % 360 + 360) % 360;
	}

	private static void rotateTo(double targetDeg) {// First approach to setting direction, should defenitely be
													// improved after testing
		double currentDeg = toTerminalAngle(targetDeg - imu.getYaw());
		if (currentDeg < 2 || currentDeg > 358) {
			leftmg.set(0);
			rightmg.set(0);
			currentTaskDone = true;
			return;
		}
		if (currentDeg > 180) {
			double degRot = 360 - currentDeg;
			System.out.println("Rotate " + degRot + " in CCW dir.");
			leftmg.set(Math.max(-degRot / 100, -1));
			rightmg.set(Math.min(degRot / 100, 1));
		} else {
			double degRot = currentDeg;
			System.out.println("Rotate " + degRot + " in CW dir.");
			leftmg.set(Math.min(degRot / 100, 1));
			rightmg.set(Math.max(-degRot / 100, -1));
		}
	}

	static boolean initDrive = true;

	private static void driveForward(double inches) {// drives forwards the set amount
		// System.out.println("Driving Forward " + inches + " inches.");
		if (initDrive) {// Reset the encoders at the intial starting point
			lEncoder.reset();
			rEncoder.reset();
		}
		initDrive = false;// finish intial set-up
		double approxDistance = (lEncoder.getDistance() + rEncoder.getDistance()) / 2;// estimate distance w/ the
																						// average of the encoders
		if (inches - approxDistance < .5 && inches - approxDistance > -.5) {
			currentTaskDone = true;// Within half an inch, declare done and set initDrive for next drive command
			initDrive = true;
			return;
		}
		leftmg.set(Math.min(1, (inches - approxDistance) / 12));// Set the motors proportional to distance
		rightmg.set(Math.min(1, (inches - approxDistance) / 12));

	}

	private static void HatchUp(Double d) {// retracts arm
		Robot.hook.setHookMotor(.11 * 3);
		if (RobotMap.upperSwitch.get() || OI.logitech.getBButton()) {// Stop Raising
			Robot.hook.setHookMotor(0);
			currentTaskDone = true;
		}
	}

	private static void HatchDown(Double d) {// releases hatch
		Robot.hook.setHookMotor(-.1392 * 2);
		if (RobotMap.lowerSwitch.get() || OI.logitech.getBButton()) {// Stop Lowering
			Robot.hook.setHookMotor(0);
			currentTaskDone = true;
		}
	}

}
