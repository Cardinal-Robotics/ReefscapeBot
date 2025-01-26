// 1/27/24 Changed kDriveEncoderPerPulse added "* .093433" to correctly relate encoder rotations to distance in meters
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.util.Units;

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
    public static final class LogConstants {
        public static final boolean kDisplayDebugData = true;
    }

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDeadband = 0.3;
    }

    public static final class DriveConstants {
        public static final File kSwerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        public static final double kMaxSpeed = 1.5;// 3.699417868457614; // m/s

        public static final Pose2d kInitialBlueRobotPose = new Pose2d(2.17, 7, Rotation2d.kZero);
        public static final Pose2d kInitialRedRobotPose = new Pose2d(15.378, 1.052, Rotation2d.k180deg);

        // PID values for translation (moving).
        public static final double kPTrans = 0.0022445;
        public static final double kITrans = 0;
        public static final double kDTrans = 0.35005;

        // PID values for rotating.
        public static final double kPAngular = 0.01;
        public static final double kIAngular = 0;
        public static final double kDAngular = 0.003225;

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
        // Ports/ID(s)
        public static final int kLeaderMotorPort = 0;
        public static final int kFollowerMotorPort = 0;
        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;

        // PID values for elevator controller
        public static final double kElevatorKp = 5;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 0;

        // Values for elevator feed-forward
        public static final double kElevatorKs = 0;
        public static final double kElevatorKg = 0.762;
        public static final double kElevatorKv = 0.762;
        public static final double kElevatorKa = 0;

        // idk
        public static final double kMaxAcceleration = 0;
        public static final double kMaxVelocity = 0;

        public static final double kElevatorGearing = 10.0;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);

        public static final double kCarriageMass = 4.0;

        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(74);
        public static final double kMinElevatorHeightMeters = 0;

        // distance per pulse = (distance per revolution) / (pulses per revolution)
        // or just => (Pi * D) / ppr
        public static final double kElevatorEncoderDistPerPulse = ((Math.PI) * (2 * kElevatorDrumRadius)) / 4096;
    }

    public static final class AlgaeMechanismConstants {
        // Ports/ID(s)
        public static final int kIntakeMotorPort = 20; // These ID(s) are made up so that the simulator won't complain.
                                                       // Make sure to configure these
        public static final int kTiltMotorPort = 22;

        // PID & feed forward for tilt motor
        public static final double kTiltFeedForward = 0; // Note to future self: you should probably use REV Hardware
        public static final double kTiltKp = 0; /////////// Client's PID tuning tool for this one.
        public static final double kTiltKi = 0;
        public static final double kTiltKd = 0;

        // Motor speeds (-100% to 100%) => (-1 to 1)
        public static final double kIntakeSpeed = 0.85;
        public static final double kReleaseSpeed = -0.85;

        // Time constants (how much time should the motors spend trying to
        // intake/release the algae).
        public static final double kIntakeTime = 1.0;
        public static final double kReleaseTime = 1.0;

        // Setpoints for tilt motor
        public static final double kTargetPointIntake = 0; // The tilt motor's set point before grabbing the algae out
                                                           // of the reef.
        public static final double kTargetPointDrive = 0; // The tilt motor's set point when driving around, preferably
                                                          // level.
        public static final double kTargetPointRelease = 0; // The tilt motor's set point when releasing the algae into
                                                            // the processor.
    }
}
