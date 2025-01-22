// 1/27/24 Changed kDriveEncoderPerPulse added "* .093433" to correctly relate encoder rotations to distance in meters
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDeadband = 0.3;
    }

    public static final class DriveConstants {
        public static final File kSwerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        public static final double kMaxSpeed = 3.699417868457614; // m/s

        public static final Pose2d kInitialBlueRobotPose = new Pose2d(2.17, 7, Rotation2d.kZero);
        public static final Pose2d kInitialRedRobotPose = new Pose2d(15.378, 1.052, Rotation2d.k180deg);

        // PID values for translation (moving).
        public static final double kPTrans = 0.0022445;
        public static final double kITrans = 0;
        public static final double kDTrans = 0.35005;

        // PID values for rotating.
        public static final double kPAngular = 0.01;
        public static final double kIAngular = 0;
        public static final double kDAngular = 0;

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kRearLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 11;
        public static final int kRearRightDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kRearLeftTurningMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 10;
        public static final int kRearRightTurningMotorPort = 7;

        public static final int kFrontLeftCancoderPort = 3;
        public static final int kRearLeftCancoderPort = 6;
        public static final int kFrontRightCancoderPort = 12;
        public static final int kRearRightCancoderPort = 9;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kRearRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kRearRightDriveEncoderReversed = true;

        public static final boolean kFrontLeftNotVersH = true;
        public static final boolean kRearLeftNotVersH = false;
        public static final boolean kFrontRightNotVersH = true;
        public static final boolean kRearRightNotVersH = false;
    }

    public static final class ElevatorConstants {
        public static final int kLeaderMotorId = 0;
        public static final int kFollowerMotorId = 0;

        public static final double kMaxAcceleration = 0;
        public static final double kMaxVelocity = 0;

        public static final double kEncoderDistancePerPulse = 0;

        // PID values for elevator controller
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        // Values for elevator feed-forward
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
    }
}
