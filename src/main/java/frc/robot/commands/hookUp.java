package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class hookUp extends Command{
    public hookUp() {
        requires(Robot.hook);
    }

    protected void initialize() {
        Robot.hook.setHookMotor(-.1);

    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end(){
        Robot.hook.setHookMotor(0);
    }

    protected void interrupted() {
        end();
    }
}