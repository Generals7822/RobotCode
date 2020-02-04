package frc.robot.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.*;
//import com.analog.adis16448.frc.*;
import java.util.ArrayList;
import java.util.function.Consumer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class learning {
    static Ultrasonic ult = new Ultrasonic(1, 1);
    boolean press = false;
    boolean last_pressed = false;
    public void activate(){
       boolean a_button = OI.logitech.getAButton();
       if(!last_pressed && a_button) {
           press = !press;
       }
       last_pressed = a_button;
    }

    public void tenin() {
        double distance = ult.getRangeInches();
        double speed = Math.abs(distance-10)/20;
        speed = Math.min(speed, .5);

        if (ult.isRangeValid() && distance > 11) {
            RobotMap.leftMotor1.set(-speed);
            RobotMap.leftMotor2.set(-speed);
            RobotMap.rightMotor1.set(speed);;
            RobotMap.rightMotor2.set(speed);
        }

        else if (ult.isRangeValid() && distance < 9) {
            RobotMap.leftMotor1.set(speed);
            RobotMap.leftMotor2.set(speed);
            RobotMap.rightMotor1.set(-speed);
            RobotMap.rightMotor2.set(-speed);
        }
        
        else {
            RobotMap.leftMotor1.set(0);
            RobotMap.leftMotor2.set(0);
            RobotMap.rightMotor1.set(0);
            RobotMap.rightMotor2.set(0);
        }
    }

}