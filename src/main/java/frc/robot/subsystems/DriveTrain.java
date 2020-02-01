/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.OI;
import frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {
  
  //DM declarations
  public static VictorSPX left1;
  public static VictorSPX left2;
  public static VictorSPX right1;
  public static VictorSPX right2;
  
  //Gyro
  private final SPI.Port sPort = SPI.Port.kOnboardCS0;
  private final Gyro gyro = new ADXRS450_Gyro(sPort);

  //Pathfollowing objects
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(AutoConstants.TRACK_WIDTH_METERS);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA);

  private PIDController leftPIDController = new PIDController(AutoConstants.OPTIMAL_DRIVE_KP, 0, 0);
  private PIDController rightPIDController = new PIDController(AutoConstants.OPTIMAL_DRIVE_KP, 0, 0);

  // Fields
  private double yVal = 0;
  private double twistVal = 0;
  private double yReduction = 0;
  private double twistReduction = 0;

  public DriveTrain() {
    left1 = new VictorSPX(RobotMappings.DMTopLeft);
    left2 = new VictorSPX(RobotMappings.DMBottomLeft);
    right1 = new VictorSPX(RobotMappings.DMTopRight);
    right2 = new VictorSPX(RobotMappings.DMBottomRight);
  }

  public void drive(double leftPower, double rightPower) {
    left1.set(ControlMode.PercentOutput, -leftPower);
    left2.set(ControlMode.PercentOutput, -leftPower);
    right1.set(ControlMode.PercentOutput, rightPower);
    right2.set(ControlMode.PercentOutput, rightPower);
  }

  public void driveVolts(double leftPower, double rightPower) {
    left1.set(ControlMode.PercentOutput, -leftPower / 12);
    left2.set(ControlMode.PercentOutput, -leftPower / 12);
    right1.set(ControlMode.PercentOutput, rightPower / 12);
    right2.set(ControlMode.PercentOutput, rightPower / 12);
  }

  public void resetEncoders() {

  }

  public double getLeftDistance() {

  }

  public double getRightDistance() {

  }

  public double getLeftSpeed() {
    
  }

  public double getRightSpeed() {

  }

  public void calibrate() {
    gyro.calibrate();
  }

  public void reset() {
    gyro.reset();
  }

  public double getAngle() {
    return -gyro.getAngle();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(getAngle(), 360));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  @Override
  public void periodic() {
    //Joystick drive
    yReduction = OI.joystickMain.trigger.get() ? 0.5 : 1;
    twistReduction = OI.joystickMain.trigger.get() ? 0.4 : 1;
    
    yVal = OI.joystickMain.getY() * yReduction;
    twistVal = OI.joystickMain.getTwist() * twistReduction;
    drive(yVal+twistVal, yVal-twistVal);

    //Odometry
    odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
  }
}