/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.*;

/**
 * Add your docs here.
 */

public class AutonomousCode {

    public static boolean aPressed = false;
    public static boolean autoMode = false;


    public static void autonomous(){
         
        boolean aBtn = OI.logitech.getAButton();

        if(aBtn&&!aPressed&&!autoMode){//set auto mode if aBtn is pressed and it is not in auto mode
            aPressed=true;
            autoMode=true;
        }
        if(aBtn&&!aPressed&&autoMode){//turn off autoMode if aButton is pressed, and it wasn't pressed on the prev
            autoMode=false;
            aPressed=true;
        }
        if(!aBtn){
            aPressed =false;
        }

        if(autoMode){
            
        }
    }
}
