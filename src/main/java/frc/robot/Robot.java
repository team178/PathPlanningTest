/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.RobotMappings;
import frc.robot.subsystems.DriveTrain;
import libs.IO.ThrustmasterJoystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  //Declare subsystems
  public static DriveTrain driveTrain;

  //Declare controllers
  public static ThrustmasterJoystick mainController = new ThrustmasterJoystick(RobotMappings.mainController);
  public static XboxController auxController = new XboxController(RobotMappings.auxController);

  //USB Camera declarations
  public static UsbCamera frontCamera;
  public static UsbCamera backCamera;

  @Override
  public void robotInit() {
    //Subsystems
    driveTrain = new DriveTrain();

    //Camera 1
    frontCamera = CameraServer.getInstance().startAutomaticCapture("Front cam", 0);
    frontCamera.setResolution(160, 90);
    frontCamera.setFPS(30);
    frontCamera.setPixelFormat(PixelFormat.kYUYV);

    //Camera 2
    backCamera = CameraServer.getInstance().startAutomaticCapture("Back cam", 1);
    backCamera.setResolution(160, 120);
    backCamera.setFPS(30);
    backCamera.setPixelFormat(PixelFormat.kYUYV);
  }
  
  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.periodic();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
