/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {

    public static class RobotMappings {
        //CAN IDs
        public static int DMTopLeft = 1;
        public static int DMBottomLeft = 2; 
        public static int DMTopRight = 3;
        public static int DMBottomRight = 4;

        //PCM
        public static int PCM = 11;
        
        //Computer
        public static int MAIN = 0;
        public static int AUX = 1;
    }

    public static class DriveConstants {

    }

    public static class AutoConstants {
        //Drive constants
        public static double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(6) * Math.PI;
        public static double ENCODER_TICKS = 1024;
        public static double TRACK_WIDTH_METERS = 7.239;
        public static double ENCODER_DPP = WHEEL_CIRCUMFRENCE / ENCODER_TICKS;

        //Pathplanning constants
        public static double MAX_VEL_MPS = 0;
        public static double MAX_ACCEL_MPSPS = 0;
        public static double OPTIMAL_DRIVE_KP = 0;
        
        public static double kS = 0;
        public static double kV = 0;
        public static double kA = 0;
        
        public static double RAMSETE_B = 0;
        public static double RAMSETE_ZETA = 0;
    }

}
