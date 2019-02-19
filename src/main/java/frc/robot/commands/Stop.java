package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Stop extends Command{
    public Stop() {
        requires(Robot.driving);
    }

    protected void initialize() {
        Robot.driving.stop();
    }

    protected void execute() {

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