//will hogan 1/31/19

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
 
  Joystick leftJoy = new Joystick(0);
  Joystick rightJoy = new Joystick(1);

  Button decreaseSensitivity = new JoystickButton(rightJoy, 2);
  Button hatchGrabber = new JoystickButton(rightJoy, 1); 


 public OI() {
  
    //links button where thumb rests on right joystick to sensetive driving mode when held
  
  decreaseSensitivity.whileHeld(new DriveSensetiveCommand());

 }
}
