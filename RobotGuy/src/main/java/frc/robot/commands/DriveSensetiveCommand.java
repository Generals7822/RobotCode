package frc.robot.commands; 

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/*
*This is the sensetive drive command that richard wanted it uses the sensetivedrive method in the drive 
subsystem:
* 
*Edits: WH 2/1/19
*/

public class DriveSensetiveCommand extends Command { 

    //constructor establishes dependencies
    public DriveSensetiveCommand() {
        requires(Robot.driveSub);
    }

    @Override
    protected void initialize(){

    }

    @Override
    protected void execute(){
        double rightVal = -1 * Robot.OI.rightJoy.getY();
        double leftVal = -1 * Robot.OI.leftJoy.getY();

        Robot.driveSub.sensitiveDrive(leftVal, rightVal);
    }

    @Override
    protected boolean isFinished(){
        return false; 
    }

    @Override
    protected void interrupted(){

    }

    @Override
    protected void end(){

    }

}
