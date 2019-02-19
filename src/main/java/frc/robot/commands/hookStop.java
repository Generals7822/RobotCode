package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class hookStop extends Command{
    public hookStop() {
        requires(Robot.hook);
    }

    protected void initialize() {
        Robot.hook.setHookMotor(0);

    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end(){
        
    }

    protected void interrupted() {
        end();
    }
}