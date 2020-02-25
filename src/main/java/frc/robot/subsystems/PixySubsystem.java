/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.*;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import java.util.logging.Logger;
//import io.github.pseudoresonance.pixy2api.*;

/**
 * Add your docs here.
 */
public class PixySubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static   {
	pixy1 = new SPI(SPI.Port.kMXP) ;
}
static SPI pixy1;
static I2C pixy2;

private static byte[] sendAndRecieve(int type, byte[]outbytes, int MaxExpectedReturnPayload){
    // Find length of desired output data packet
    // If no packet supplied, then use zero 

    int length;

    if (outbytes == null){
        length = 0;
    }else{
        length = outbytes.length ;
    }

    //System.out.println( "sr out length =" + length) ;
    // Prepare initial 4 bytes of outgoing data packet

    byte[] myOut = new byte[length + 4];
    myOut[0] = (byte)174;
    myOut[1] = (byte)193;
    myOut[2] = (byte)type;
    myOut[3] = (byte)length;

    if(length > 0){
        for(int i = 0; i < length; i++){
            myOut[4 + i] = outbytes[i];
        }
    }
   //byte[] getver = new byte[]  {(byte) 174, (byte) 193, 14, 0};   
   //boolean bok = mI2C.writeBulk(getver, 4) ;

   int bok = pixy1.write(myOut, myOut.length);


    //System.out.println("write result" + bok + "myout.length =" + myOut.length) ;
    byte [] retval = new byte[ MaxExpectedReturnPayload + 6];

    //bok = pixy1.readOnly(retval, retval.length) ;

    System.out.println ("retval length" + retval.length) ;

    //mI2C.close(); 

    return retval ;
    }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setDistance()
  {
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

  private static double toTerminalAngle(double angle) {// Converts to Terminal Angle
		return (angle % 360 + 360) % 360;
	}

	
	
}