/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.OI;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.PixyCamBlock;
import edu.wpi.first.wpilibj.Victor;

public class PixyAlignSubsystem extends Subsystem {
  
  @Override
  public void initDefaultCommand() {
  
  }

	public static Victor elevator1= RobotMap.elevatorMotor;
	public static Victor shooter1= RobotMap.shootMotor; 
	public static SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motors Intiaited
	public static SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
	
	public static void autoShoot()
	{
		shooter1.set(-0.6);
	}

	public static void unjam()
	{
		elevator1.set(0.5);
		shooter1.set(0.75);
	}

	public static void autonomousStart(SpeedControllerGroup lmg, SpeedControllerGroup rmg, double power, int direction)
  	{
		lmg.set(Math.max(-power+direction, -1));
    	rmg.set(Math.min(-direction-power,1));
  	}
  	public static void timedDistance(SpeedControllerGroup lmg, SpeedControllerGroup rmg, Timer t)
  	{
		Timer timer= t; 
		if(timer.get()<0.5)
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
	public static void findTarget(SpeedControllerGroup lmg, SpeedControllerGroup rmg)
	{
		// lmg.set(-0.2);
		// rmg.set(-0.2);
		// PixyCamBlock[] blocks = George_Pixy.getBlocks();
		// if(blocks != null && blocks.length != 0)
		// testpixy();
		// for(int i=0; i<blocks.length; i++)
		// {
		// 	if(blocks[i].yCenter<100)
		// 	{
		// 		testpixy();
		// 	}
		// }
	}
	public static void testpixy() {
		if(OI.logitech.getBumper(Hand.kLeft))
		{
		PixyCamBlock[] blocks = George_Pixy.getBlocks();
		
		double addRight;
		double addLeft;
		if (blocks != null && blocks.length != 0) 
		{
			double currentDeg = toTerminalAngle(160 - blocks[0].xCenter);
			if (currentDeg < 5 || currentDeg > 355) 
			{// If it is within 5 degrees, stop
				addRight = 0;
				addLeft = 0;
				// currentTaskDone = true;
				// return;
			}
			if (currentDeg > 180) 
			{
				double degRot = 360 - currentDeg;
				// System.out.println("Rotate " + degRot + " in CCW dir.");
				addLeft = -Math.max(-degRot / 500, -0.5);
				addRight = Math.min(degRot / 500, 0.5);
			} 
			else 
			{
				double degRot = currentDeg;
				// System.out.println("Rotate " + degRot + " in CW dir.");
				addLeft = -Math.min(degRot / 500, 0.5);
				addRight = Math.max(-degRot / 500, -0.5);
			}

			if (blocks[0].height <= 45) 
			{
				RobotMap.leftMotor1.set(-.25 + addLeft);
				RobotMap.leftMotor2.set(-.25 + addLeft);
				RobotMap.rightMotor1.set(.25 + addRight);
				RobotMap.rightMotor2.set(.25 + addRight);
			} 
			else if (blocks[0].height >= 55) 
			{
				RobotMap.leftMotor1.set(.25 + addLeft);
				RobotMap.leftMotor2.set(.25 + addLeft);
				RobotMap.rightMotor1.set(-.25 + addRight);
				RobotMap.rightMotor2.set(-.25 + addRight);
			} 
			else 
			{
				RobotMap.leftMotor1.set(0);
				RobotMap.leftMotor2.set(0);
				RobotMap.rightMotor1.set(0);
				RobotMap.rightMotor2.set(0);
				//Robot.isCenter=true;
			}	

		} 

		else
		{
			RobotMap.leftMotor1.set(-0.2);
			RobotMap.leftMotor2.set(-0.2);
			RobotMap.rightMotor1.set(-0.2);
			RobotMap.rightMotor2.set(-0.2);
		}
		}
	}

	

// 	private static void rotateBy(double targetDeg) 
// 	{
// //First approach to setting direction, should defenitely be
// 												// improved after testing
												
// 		double currentDeg = toTerminalAngle(targetDeg);
// 		// SmarthDasboard.putNumber("Yaw", imu.getYaw());
// 		// SmartDashboard.putNumber("XAngle", imu.getAngleX());
// 		// SmartDashboard.putNumber("YAngle", imu.getAngleY());
// 		// SmartDashboard.putNumber("ZAngle", imu.getAngleZ());

// 		if (currentDeg < 5 || currentDeg > 355) {// If it is within 5 degrees, stop
// 			leftmg.set(0);
// 			rightmg.set(0);
// 			// currentTaskDone = true;
// 			// return;
// 		}
// 		if (currentDeg > 180) {
// 			double degRot = 360 - currentDeg;
// 			// System.out.println("Rotate " + degRot + " in CCW dir.");
// 			leftmg.set(-Math.max(-degRot / 500, -0.5));
// 			rightmg.set(Math.min(degRot / 500, 0.5));
// 		} else {
// 			double degRot = currentDeg;
// 			// System.out.println("Rotate " + degRot + " in CW dir.");
// 			leftmg.set(-Math.min(degRot / 500, 0.5));
// 			rightmg.set(Math.max(-degRot / 500, -0.5));
// 		}
		
// 	}
// 	static boolean sAPressed = false;
// 	static boolean sSlowmode = false;

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
	
// 	private static void rotateDrive(double targetDeg) 
// 	{
// //First approach to setting direction, should defenitely be
// 												// improved after testing
												
// 		double currentDeg = toTerminalAngle(targetDeg);
// 		// SmarthDasboard.putNumber("Yaw", imu.getYaw());
// 		// SmartDashboard.putNumber("XAngle", imu.getAngleX());
// 		// SmartDashboard.putNumber("YAngle", imu.getAngleY());
// 		// SmartDashboard.putNumber("ZAngle", imu.getAngleZ());

// 		if (currentDeg < 5 || currentDeg > 355) {// If it is within 5 degrees, stop
// 			leftmg.set(0);
// 			rightmg.set(0);
// 			// currentTaskDone = true;
// 			// return;
// 		}
// 		if (currentDeg > 180) {
// 			double degRot = 360 - currentDeg;
// 			// System.out.println("Rotate " + degRot + " in CCW dir.");
// 			leftmg.set(-Math.max(-degRot / 500, -0.5));
// 			rightmg.set(Math.min(degRot / 500, 0.5));
// 		} else {
// 			double degRot = currentDeg;
// 			// System.out.println("Rotate " + degRot + " in CW dir.");
// 			leftmg.set(-Math.min(degRot / 500, 0.5));
// 			rightmg.set(Math.max(-degRot / 500, -0.5));
// 		}
		
//   }
  
  private static double toTerminalAngle(double angle) {// Converts to Terminal Angle
		return (angle % 360 + 360) % 360;
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





}

