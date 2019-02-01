package main.java.frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveManuallyCommand extends Command {

    public DriveManuallyCommand() {
        requires(robot.driveSub);
    }

    @Override
    protected void initialize(){
        
    }

    //called every 10ms this is the meat and potatoes baby
    @Override
    protected void execute(){
        double rightVal = -1 * Robot.OI.rightJoy.getY();
        double leftVal = -1 * Robot.OI.leftJoy.getY();

        Robot.driveSub.manualDrive(leftVal, rightVal);
    }

    //returns true when its done
    @Override
    protected boolean isFinished() {
        return false; 
    }

    @Override 
    protected void end() {

    }

    @Override
    protected void interrupted() {

    }

}