/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
    
    public static class RobotMappings {
        //Robot
        public static int PCM = 0;
        public static int DMLeftMaster = 1;
        public static int DMLeftSlave = 2;
        public static int DMRightMaster = 3;
        public static int DMRightSlave = 4;
        public static int encoderFeedbackDevice = 0;

        public static SPI.Port gyroPort = SPI.Port.kOnboardCS0;
        
        //Computer
        public static int mainController = 0;
        public static int auxController = 1;
    }

    public static class PathConstants {
        //Misc
        public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
        public static final double kTrackWidthMeters = Units.inchesToMeters(23.75);
        public static final double kEncoderPPR = 4096;
        public static final double kEncoderDPP =  kWheelDiameterMeters * Math.PI / kEncoderPPR;

        //Parameters
        public static final double kMaxVelMPS = 0;
        public static final double kMaxAccelMPSPS = 3;
        public static final double kMaxVoltage = 10;

        //Feedforward gains
        public static final double kS = 0.898;
        public static final double kV = 0.772;
        public static final double kA = 0.091;

        //Feedback gains
        public static final double kDriveP = 4.09;

        //Ramsete
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
