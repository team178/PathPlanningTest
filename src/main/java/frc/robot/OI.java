/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;
import libs.IO.ThrustmasterJoystick;

/**
 * Add your docs here.
 */
public class OI {

    public static ThrustmasterJoystick joystickMain = new ThrustmasterJoystick(RobotMappings.MAIN);
    public static XboxController xboxAux = new XboxController(RobotMappings.AUX);
    
}
