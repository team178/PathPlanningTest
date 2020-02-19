/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

  //Motor controllers
  private static WPI_TalonSRX leftMaster;
  private static WPI_VictorSPX leftSlave;
  private static WPI_TalonSRX rightMaster;
  private static WPI_VictorSPX rightSlave;

  private static SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMaster, leftSlave);
  private static SpeedControllerGroup rightMotors = new SpeedControllerGroup(leftMaster, leftSlave);

  //Encoders 
  private final int TIMEOUT = 30;//"block" to stop the config from continuously updating until success (not really sure why we need this -Darren)
  private static int distanceTravelled; 

  //Gyro
  private final Gyro gyro = new ADXRS450_Gyro(RobotMappings.gyroPort);

  //Pathplanning
  private static DifferentialDriveKinematics kinematics;
  private static DifferentialDriveOdometry odometry;
  private static SimpleMotorFeedforward feedforward;

  private static PIDController leftPIDController;
  private static PIDController rightPIDController;

  private static double yVal;
  private static double twistVal;
  private static double yReduction;
  private static double twistReduction;
  
  public DriveTrain() {
    leftMaster = new WPI_TalonSRX(RobotMappings.DMLeftMaster);
    leftSlave = new WPI_VictorSPX(RobotMappings.DMLeftSlave);
    rightMaster = new WPI_TalonSRX(RobotMappings.DMRightMaster);
    rightSlave = new WPI_VictorSPX(RobotMappings.DMRightSlave);
    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    setDriveDirection(DriveDirection.FORWARD);

    kinematics = new DifferentialDriveKinematics(PathConstants.kTrackWidthMeters);
    odometry = new DifferentialDriveOdometry(getAngle());
    feedforward = new SimpleMotorFeedforward(PathConstants.kS, PathConstants.kV, PathConstants.kA);

    leftPIDController = new PIDController(PathConstants.kDriveP, 0, 0);
    leftPIDController = new PIDController(PathConstants.kDriveP, 0, 0);

    //Encoders 
    //sets the "base" of the relative quadulature measurement to the abolute measurement from the pulse width magnet measurement 
    //initQuadulature(leftMaster);
    //initQuadulature(rightMaster);
    //initQuadulature(leftSlave);
    //initQuadulature(rightSlave);
  }

  public void drive(double leftPower, double rightPower) {
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  public void setDriveDirection(DriveDirection driveDirection) {
    if (driveDirection == DriveDirection.FORWARD) {
      leftMaster.setInverted(false);
      rightMaster.setInverted(true);
    } else {
      leftMaster.setInverted(true);
      rightMaster.setInverted(false);
    }
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftRate(),
      getRightRate()
    );
  }

  /*
   * Encoder methods
   * Encoder gets "Quadulature" and "Pulse Width (Magnet)" value 
   * - Quadulature is fast updating but relative 
   * - Pulse Width is absoulute, but slow 
   * - Basically, just assign the starting vaue of Quadulature to Pulse Width 
   */

  public void initQuadulature(WPI_TalonSRX talonSRX)//assigns absolute value from magnet to starting val of quadulature 
  {
    int pulseWidth = talonSRX.getSensorCollection().getPulseWidthPosition();//gets pulse width (absolute) value 
    //disconuity with books ends goes here, if we need it 
    pulseWidth = pulseWidth & 0xFFF;//makes sure value is within 0 and 360 degrees
    talonSRX.getSensorCollection().setQuadraturePosition(pulseWidth, TIMEOUT);//assigns starting quadrature position to pulse width 
    //getSensorCollection() returns an object that can get raw data 
  }

  public double getDistance(WPI_TalonSRX talonSRX)
  {
    int temp = distanceTravelled;
    distanceTravelled = 0; 
    return temp; 
  }

  public double getPosition(WPI_TalonSRX talon) {
    //getPulseWidthPosition() returns a value from -4096 to 4096, this converts it to degrees 
    return talon.getSensorCollection().getPulseWidthPosition() * 360.0 / 4096.0;
  }

  public double getRelativeVelocity(WPI_TalonSRX talon) {
    return talon.getSensorCollection().getPulseWidthVelocity();
  }

  public double getAbsoluteVelocity(WPI_TalonSRX talon) {
    return talon.getSensorCollection().getQuadratureVelocity();
  }

  public double getLeftDistance() {
    return 0;
  }

  public double getRightDistance() {
    return 0;
  }

  public double getLeftRate() {
    return 0;
  }

  public double getRightRate() {
    return 0;
  }

  public void resetEncoders() {

  }

  /*
   * gyro methods
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }
  
  public void calibrateGyro() {
    gyro.calibrate();
  }

  public void resetGyro() {
    gyro.reset();
  }

  /*
   * odometry methods
   */
  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getAngle());
  }

  @Override
  public void periodic() {
    //Joystick drive
    yReduction = Robot.mainController.trigger.get() ? 0.5 : 0.8;
    twistReduction = Robot.mainController.trigger.get() ? 0.3 : 0.5;

    yVal = Robot.mainController.getY() * yReduction;
    twistVal = Robot.mainController.getTwist() * twistReduction;

    drive(yVal+twistVal, yVal-twistVal);

    //Path planning
    odometry.update(getAngle(), getLeftDistance(), getRightDistance());

    //test
    System.out.println("left dist: " + getDistance(leftMaster));
    System.out.println("right dist: " + getDistance(rightMaster));
  }

  /*
   * path planning accessors
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
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

  enum DriveDirection {
    FORWARD,
    BACKWARD
  }
}
