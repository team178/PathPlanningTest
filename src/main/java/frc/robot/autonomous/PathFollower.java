/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveTrain;

public class PathFollower {

    private static DriveTrain driveTrain = Robot.driveTrain;
    
    public static Command getAutonomousCommand() {
        //Creating a voltage constraint object
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA),
            driveTrain.getKinematics(),
            12
        );

        //Create trajectory config and apply voltage constraint
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.MAX_VEL_MPS,
            AutoConstants.MAX_ACCEL_MPSPS
        )
        .setKinematics(driveTrain.getKinematics())
        .addConstraint(autoVoltageConstraint);
        
        //Nithin's test trajectory
        Trajectory nithinsTestTrajectory = TrajectoryGenerator.generateTrajectory(
            //Start pose
            new Pose2d(0, 0, new Rotation2d(0)),
         
            //Interior waypoints
            List.of(
                new Translation2d(1, 1),
                new Translation2d(-0.25, 4.5),
                new Translation2d(-3, 4),
                new Translation2d(-5, 7),
                new Translation2d(-5.5, 7.25)
            ),
         
            //End pose
            new Pose2d(-6, 8, new Rotation2d(53.14)),
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
            nithinsTestTrajectory,
            driveTrain::getPose,
            new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
            driveTrain.getFeedforward(),
            driveTrain.getKinematics(),
            driveTrain::getWheelSpeeds,
            driveTrain.getLeftPIDController(),
            driveTrain.getRightPIDController(), 
            driveTrain::tankDriveVolts,
            driveTrain
        );

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> driveTrain.driveVolts(0, 0));
        }
    }
}
