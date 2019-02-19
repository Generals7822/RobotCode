package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;

public class SensitiveDriving extends Command{
    public SensitiveDriving() {
        requires(Robot.driving);
    }

    protected void initialize() {
        
    }

    protected void execute() {
       // Robot.driving.squareSpeed(.5*OI.leftJoy.getY(), .5*OI.rightJoy.getY());
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end(){
        Robot.driving.stop();
    }

    protected void interrupted() {
        end();
    }
}