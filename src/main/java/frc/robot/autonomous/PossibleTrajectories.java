/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Robot;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.DriveTrain;
/**
 * Add your docs here.
 */
public class PossibleTrajectories {

    private DriveTrain driveTrain = Robot.driveTrain;

    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        driveTrain.getFeedforward(), 
        driveTrain.getKinematics(), 
        PathConstants.kMaxVoltage
    );

        //Creating trajectory config
        TrajectoryConfig configForward = new TrajectoryConfig(
        PathConstants.kMaxVelMPS, 
        PathConstants.kMaxAccelMPSPS
        )
        .setKinematics(driveTrain.getKinematics())
        .setReversed(false)
        .addConstraint(voltageConstraint);

    //Note: The distances and angles are not correct and are just educated guesses
    

        //Creating Trajectory for middle starting position going forwards
        Trajectory TrajectoryMiddleForward = TrajectoryGenerator.generateTrajectory(
            //Start pose
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1.5,0)
                ),
            // End pose
            new Pose2d(3, 0, new Rotation2d(0)),
            configForward
        );

        //Creating Trajectory for Right position going forwards
        Trajectory TrajectoryRightForward = TrajectoryGenerator.generateTrajectory(
            //Start pose
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1.5, -0.2) //Insert proper dimensions
                ),
            // End pose
            new Pose2d(3, -0.4, new Rotation2d(30)), //Insert proper dimensions
            configForward
        );

        //Creating Trajectory for left position going forwards
        Trajectory TrajectoryLeftForward = TrajectoryGenerator.generateTrajectory(
            //Start pose
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1.5, 0.2)
            ),
            // End pose
            new Pose2d(3, 0.4, new Rotation2d(-30)),
            configForward
        );

        //Creating trajectory configBackwards
        TrajectoryConfig configBackward = new TrajectoryConfig(
        PathConstants.kMaxVelMPS, 
        PathConstants.kMaxAccelMPSPS
        )
        .setKinematics(driveTrain.getKinematics())
        .setReversed(true)
        .addConstraint(voltageConstraint);

        //Creating Trajectory for middle position going backwards
        Trajectory TrajectoryMiddleBack = TrajectoryGenerator.generateTrajectory(
            //Start pose
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1.5,0)
                ),
            // End pose
            new Pose2d(3, 0, new Rotation2d(0)),
            configBackward
            );

        //Creating Trajectory for right position going backwards
        Trajectory TrajectoryRightBack = TrajectoryGenerator.generateTrajectory(
            //Start pose
            new Pose2d(0, 0, new Rotation2d(30.0)),
            List.of(
                new Translation2d(1.5,-1)
                ),
            // End pose
            new Pose2d(3, 2, new Rotation2d(45.0)),
            configBackward
        );

        //Creating Trajectory for left position going backwards
        Trajectory TrajectoryLeftBack = TrajectoryGenerator.generateTrajectory(
            //Start pose
            new Pose2d(0, 0, new Rotation2d(30.0)),
            List.of(
                new Translation2d(1.5,-2)
                ),
            // End pose
            new Pose2d(3, -2, new Rotation2d(45.0)),
            configBackward
        );
}
