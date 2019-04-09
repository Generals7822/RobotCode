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

/**
 * Add your docs here.
 */

public class AutonomousCode { //All ToDo Functions must take in a double and be void, all changes should be made through field variables

    public static boolean aPressed = false;
    public static boolean autoMode = false;
    public static ADIS16448_IMU imu = new ADIS16448_IMU();
    public static Ultrasonic ult = new Ultrasonic(1,1);//Set Actual Input and Output values
	
	public static void initAuto(){
		FuncsToDo.clear();
		inputs.clear();
        double robotAngle = toTerminalAngle(imu.getYaw());
		if(robotAngle>180) {
			robotAngle = robotAngle-360;
		}
		
		double hatchAngle = robotAngle+getHatchAngle();//Comes from PixyCam
		
		
		boolean driveToRight = hatchAngle>0;//Sees if we are driving to the left or right
		hatchAngle = Math.abs(hatchAngle);//changes everything to positive
		double shipDistance = ult.getRangeInches()*Math.cos(robotAngle*Math.PI/180); //distance of Robot to Cargo Ship
		double hatchAngleRad = Math.PI*hatchAngle/180;//Convert to Radians
		double inchesFromDrop = 2; //how far in front the robot should be of the cargo ship when lining up
		
		
		double pathDistance = Math.sqrt(shipDistance*shipDistance
				*Math.tan(hatchAngleRad)*Math.tan(hatchAngleRad)+(shipDistance-inchesFromDrop)
				*(shipDistance-inchesFromDrop)); //Formula for calculating the distance to the spot in front of the cargo ship
		double pathTurnAngle = 180/Math.PI*(hatchAngleRad+Math.acos(-(-2*shipDistance
				+inchesFromDrop*Math.cos(2*hatchAngleRad)+inchesFromDrop)
				/(2*Math.cos(hatchAngleRad)*pathDistance)));//Formula for calculating angle to travel in

				FuncsToDo.add(AutonomousCode::rotateTo);
				if(driveToRight) {
					//If we are going right, go right
					inputs.add(pathTurnAngle);
				}else{
					inputs.add(360-pathTurnAngle);//If we are going left, go left
				}
				FuncsToDo.add(AutonomousCode::driveForward);
				inputs.add(pathDistance);

				FuncsToDo.add(AutonomousCode::rotateTo);//rotate towards the ship
				inputs.add(0.0);

				FuncsToDo.add(AutonomousCode::driveForward);
				inputs.add(2.0);

				FuncsToDo.add(AutonomousCode::releaseHatch);
				inputs.add(0.0);

	}
	
	static int step = 0;
	static boolean toDoInProgress = false;

    public static void autonomous(){
         
        boolean aBtn = OI.logitech.getAButton();

        if(aBtn&&!aPressed&&!autoMode){//set auto mode if aBtn is pressed and it is not in auto mode
            aPressed=true;
			autoMode=true;
			step=0;//step counter for future additions to autonomous
        }
        if(aBtn&&!aPressed&&autoMode){//turn off autoMode if aButton is pressed, and it wasn't pressed on the prev
            autoMode=false;
			aPressed=true;
			toDoInProgress = false;

        }
        if(!aBtn){
            aPressed = false;
        }

        if(autoMode){
            if(toDoInProgress){
				ToDo(FuncsToDo, inputs);
			}else{
				initAuto();
			}
        }
	}
	static ArrayList<Consumer<Double>> FuncsToDo = new ArrayList<>();
	static ArrayList<Double> inputs = new ArrayList<>();

	static int currentTaskNum = 0;
	static boolean currentTaskDone = false;

	private static boolean ToDo(ArrayList<Consumer<Double>> funcs, ArrayList<Double> inputs) {
		funcs.get(currentTaskNum).accept(inputs.get(currentTaskNum));
		while(currentTaskDone) {
			currentTaskNum++;
			currentTaskDone=false;
			System.out.println(currentTaskNum);
			if(currentTaskNum==funcs.size()) {
				return true;
			}
			funcs.get(currentTaskNum).accept(inputs.get(currentTaskNum));
		}
		
		return false;
	}

    public static double getHatchAngle(){
		return 0;
		//Complete w/ PixyCam
    }

    private static double toTerminalAngle(double angle) {
		return (angle%360+360)%360;
	}

	private static void rotateTo(double targetDeg) {
		double currentDeg = toTerminalAngle(targetDeg - imu.getYaw());
		if (currentDeg > 180) {
			double degRot = 360-currentDeg;
			System.out.println("Rotate "+degRot+" in CCW dir.");
			// rotate by degRot in a counterclockwise direction
		} else {
			double degRot = currentDeg;
			System.out.println("Rotate "+degRot+" in CW dir.");
			// rotate by degRot in a clockwise direction
		}
	}
	
	private static void driveForward(double inches) {//drives forwards the set amount
		System.out.println("Driving Forward "+inches+" inches.");
		//Complete

	}
	public static boolean UpState = true;
	public static boolean DownState = false;
	// Called repeatedly when this Command is scheduled to run
	static boolean goingDown = false;
	static boolean goingUp = false;
	static long time = 0;

	private static void releaseHatch(Double d) {//releases hatch
		goingUp = true;
		Robot.hook.setHookMotor(.11*3);
		time = System.currentTimeMillis();//Gets current time
		  
		boolean timeOut = System.currentTimeMillis()-time>2000;//Stop if it takes longer than 2 sec
		if(goingUp&&(RobotMap.upperSwitch.get()||OI.logitech.getBButton()||timeOut)){//Stop Raising
			Robot.hook.setHookMotor(0);
			DownState = false;
			UpState = true;
			goingUp = false;
		}
	}
    

    

    
}
