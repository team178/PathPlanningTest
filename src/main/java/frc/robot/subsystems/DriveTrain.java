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

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
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
  private static TalonSRX leftMaster;
  private static VictorSPX leftSlave;
  private static TalonSRX rightMaster;
  private static VictorSPX rightSlave;

  //Encoders 
  private final int TIMEOUT = 30;//"block" to stop the config from continuously updating until success (not really sure why we need this -Darren)
  
  private final Gyro gyro = new ADXRS450_Gyro(RobotMappings.gyroPort);

  //Pathplanning
  private static DifferentialDriveKinematics kinematics;
  private static DifferentialDriveOdometry odometry;
  private static SimpleMotorFeedforward feedforward;

  private static PIDController leftPIDController;
  private static PIDController rightPIDController;

  private double yVal;
  private double twistVal;
  private double yReduction;
  private double twistReduction;
  
  public DriveTrain() {
    leftMaster = new TalonSRX(RobotMappings.DMLeftMaster);
    leftSlave = new VictorSPX(RobotMappings.DMLeftSlave);
    rightMaster = new TalonSRX(RobotMappings.DMRightMaster);
    rightSlave = new VictorSPX(RobotMappings.DMRightSlave);
    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    rightMaster.setInverted(true);
    rightSlave.setInverted(InvertType.FollowMaster);

    kinematics = new DifferentialDriveKinematics(PathConstants.kTrackWidthMeters);
    odometry = new DifferentialDriveOdometry(getAngle());
    feedforward = new SimpleMotorFeedforward(PathConstants.kS, PathConstants.kV, PathConstants.kA);

    leftPIDController = new PIDController(PathConstants.kDriveP, 0, 0);
    leftPIDController = new PIDController(PathConstants.kDriveP, 0, 0);

    //Encoders 
    //sets the "base" of the relative quadulature measurement to the abolute measurement from the pulse width magnet measurement 
    initQuadulature(leftMaster);
    initQuadulature(rightMaster);
    //initQuadulature(leftSlave);
    //initQuadulature(rightSlave);
  }

  public void drive(double leftPower, double rightPower) {
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftMaster.set(ControlMode.PercentOutput, leftVolts / PathConstants.kMaxVoltage);
    rightMaster.set(ControlMode.PercentOutput, rightVolts / PathConstants.kMaxVoltage );
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

  public void initQuadulature(TalonSRX talonSRX)//assigns absolute value from magnet to starting val of quadulature 
  {
    int pulseWidth = talonSRX.getSensorCollection().getPulseWidthPosition();//gets pulse width (absolute) value 
    //disconuity with books ends goes here, if we need it 
    pulseWidth = pulseWidth & 0xFFF;//makes sure value is within 0 and 360 degrees
    talonSRX.getSensorCollection().setQuadraturePosition(pulseWidth, TIMEOUT);//assigns starting quadrature position to pulse width 
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
    if (Robot.mainController.headBottom.get()) {
      resetGyro();
    }

    yReduction = Robot.mainController.trigger.get() ? 0.5 : 1;
    twistReduction = Robot.mainController.trigger.get() ? 0.4 : 1;

    yVal = Robot.mainController.getY() * yReduction;
    twistVal = Robot.mainController.getTwist() * twistReduction;

    drive(yVal+twistVal, yVal-twistVal);

    //Path planning
    odometry.update(getAngle(), getLeftDistance(), getRightDistance());
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
}
