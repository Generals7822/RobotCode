/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class CustomPIDTest {//Based on https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html

        double P = 1;
        double I = 0;
        double D = 0;
        double integral, previous_error, setpoint = 0;
        double rcw;
        Encoder lEncoder = new Encoder(0,1);
        Encoder rEncoder = new Encoder(2,3);
    
    
        public CustomPIDTest(int distance){
            setpoint = distance;
        }
    
        public void PID(){
            double error = setpoint +(lEncoder.getDistance()+rEncoder.getDistance())/2; // Error = Target - Actual
            this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
            double derivative = (error - this.previous_error) / .02;
            this.rcw = P*error + I*this.integral + D*derivative;
        }
    
        public void execute()
        {
            PID();

        }
    }

