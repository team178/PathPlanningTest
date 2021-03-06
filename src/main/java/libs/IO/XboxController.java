/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package libs.IO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Add your docs here.
 */
public class XboxController {

    public Joystick xbox;
    public Button auxA;
    public Button auxB;
    public Button auxX;
    public Button auxY;
    public Button auxLeftBumper;
    public Button auxRightBumper;
    public Button auxBack;
    public Button auxStart;

    private int port;

    public XboxController(int port) {
        xbox = new Joystick(port);
        auxA = new JoystickButton(xbox, 1);
        auxB = new JoystickButton(xbox, 2);
        auxX = new JoystickButton(xbox, 3);
        auxY = new JoystickButton(xbox, 4);
        auxLeftBumper = new JoystickButton(xbox, 5);
        auxRightBumper = new JoystickButton(xbox, 6);
        auxBack = new JoystickButton(xbox, 7);
        auxStart = new JoystickButton(xbox, 8);

        this.port = port;
    }

    public double getLeftStickX() {
	    return xbox.getRawAxis(0);
    }
	
    public double getLeftStickY() {
	    return xbox.getRawAxis(1);
    }
	
    public double getRightStickX() {
	    return xbox.getRawAxis(4);
    }
	
    public double getRightStickY() {
	    return xbox.getRawAxis(5);
    }
	
    public double getLeftTrigger() {
	    return xbox.getRawAxis(2);
    }
	
    public double getRightTrigger() {
	    return xbox.getRawAxis(3);
    }

    public int getPort() {
        return port;
    }
}
