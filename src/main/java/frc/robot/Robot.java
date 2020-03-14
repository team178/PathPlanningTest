/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import libs.IO.XboxController;
import libs.limelight.LimelightCamera;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotMappings;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.TestAlternateSubsystem;
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
  public static TestAlternateSubsystem motor;
  public static LimelightCamera limelight;

  //Declare controllers
  public static ThrustmasterJoystick mainController;
  public static XboxController auxController;

  //USB Camera declarations
  public static MjpegServer server;
  public static UsbCamera frontCamera;
  public static UsbCamera backCamera;
  public static boolean toggleSwap;
  public static boolean toggleBuffer;
  
  //Declare auto sendable choosers
  public static SendableChooser<String> startPath = new SendableChooser<>();
  public static SendableChooser<String> endLocation = new SendableChooser<>();
  
  @Override
  public void robotInit() {

    startPath.addOption("Left","Left");
    startPath.addOption("Middle","Middle");
    startPath.addOption("Right","Right");

    endLocation.addOption("Left","Left");
    endLocation.addOption("Middle","Middle");
    endLocation.addOption("Right","Right");


    SmartDashboard.putData("AutoLocation", startPath);
    SmartDashboard.putData("AutoLocation", endLocation);
    
    //Subsystems
    driveTrain = new DriveTrain();
    motor = new TestAlternateSubsystem();
    limelight = new LimelightCamera();

    //Camera Server declaration]
    
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

    server = CameraServer.getInstance().addSwitchedCamera("server");
    server.setSource(frontCamera);

    mainController = new ThrustmasterJoystick(RobotMappings.mainController);
    auxController = new XboxController(RobotMappings.auxController);

    toggleSwap = false;
    toggleBuffer = true;
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
      CommandScheduler.getInstance().run();
      
      //Might scrw up if keep changing startPath and endLocation
      if (startPath.getSelected() != "" && endLocation.getSelected() != "" ){
        String path1 = startPath.getSelected();
        String path2 = endLocation.getSelected();
        if (path1.equals("Left")) {
          //put trajectory 1
        }
        else if (path1.equals("Middle")) {
          //put trajectory 2
        }
        else if (path1.equals("Right")) {
          //put trajectory 3
        }
        if (path2.equals("Left")) {
          //put trajectory 4
        }
        else if (path2.equals("Middle")) {
          //put trajectory 5
        }
        else if (path2.equals("Right")) {
          //put trajectory 6
        }
    } 
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.periodic();
    SmartDashboard.putBoolean("Target found", limelight.isTargetFound());
    SmartDashboard.putNumber("tx", limelight.getHorizontalDegToTarget());
    SmartDashboard.putNumber("ty", limelight.getVerticalDegToTarget());
    SmartDashboard.putNumber("area", limelight.getTargetArea());
    swapCams();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void swapCams() {
    if (auxController.auxA.get() && toggleBuffer) {
      if (!toggleSwap) {
        System.out.println("Setting Camera 2");
        server.setSource(backCamera);
        toggleSwap = true;
        toggleBuffer = false;
      } else { 
        System.out.println("Setting Camera 1");
        server.setSource(frontCamera);
        toggleSwap = false;
        toggleBuffer = false;
     }
    }
    if (!auxController.auxA.get() && !toggleBuffer) {
      toggleBuffer = true;
    }
  }

}
