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
        public static final double kMaxSpeed = 3.699417868457614; // m/s //3.4619388492 new value

        public static final Pose2d kInitialBlueRobotPose = new Pose2d(7.469, 7.457, Rotation2d.k180deg);
        public static final Pose2d kInitialRedRobotPose = new Pose2d(10.079, 0.595, Rotation2d.kZero);

        // PID values for translation (moving).
        public static final double kPTrans = 5;
        public static final double kITrans = 0;
        public static final double kDTrans = 0;// 0.35005;

        // PID values for rotating.
        public static final double kPAngular = 5;
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

    public static final class ClimberConstants {
        // Ports/ID(s)
        public static final int kFollowerMotorId = 22;
        public static final int kLeaderMotorId = 21;
        public static final int kEncoderId = 8;

        // PID values for climber
        public static final double kClimberP = 0;
        public static final double kClimberI = 0;
        public static final double kClimberD = 0;

        // Positions
        public static final double kCrushingFrame = 0;
        public static final double kCrushingCage = 0;
    }

    public static final class ElevatorConstants {
        // Ports/ID(s)
        public static final int kMasterMotorId = 15;
        public static final int kSlaveMotorId = 16;

        // PID values for elevator controller
        public static final double kElevatorP = 1.5;
        public static final double kElevatorI = 0;
        public static final double kElevatorD = 0;

        // Feedforward values (if needed)
        public static final double kElevatorKs = 0;
        public static final double kElevatorKg = 0;
        public static final double kElevatorKv = 0;

        public enum ElevatorTarget {
            CoralIntake(Double.NaN, 0.533),
            AlgaeScore(0.32, Double.NaN),
            L1(Double.NaN, 0.125),
            L2(0.88, 0.23),
            L3(1.295, 0.62),
            L4(Double.NaN, 1.295);

            private final double algaePosition;
            private final double coralPosition;

            ElevatorTarget(double algaePosition, double coralPosition) {
                this.algaePosition = algaePosition;
                this.coralPosition = coralPosition;
            }

            public double getAlgaePosition() {
                return algaePosition;
            }

            public double getCoralPosition() {
                return coralPosition;
            }
        }

        public static final double kPositionConversionFactor = 0.006985 / 0.6009005308151245 // (1/20) *
                * (0.6009005308151245 + Units.inchesToMeters(24 + (1 / 16)));// Units.inchesToMeters(62) /
        // 89.12281446531414986;
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;// (Math.PI *
                                                                                              // kElevatorPulleyDiameter)
        // / (kElevatorGearRatio * 60);
    }

    public static final class AlgaeMechanismConstants {
        // Ports/ID(s)
        public static final int kIntakeMotorPort = 17;
        public static final int kTiltMotorPort = 19;

        // PID & feed forward for tilt motor
        public static final double kTiltFeedForward = 0; // Note to future self: you should probably use REV Hardware
        public static final double kTiltKp = 0.0175; /////////// Client's PID tuning tool for this one.
        public static final double kTiltKi = 0;
        public static final double kTiltKd = 0.00145;

        // Motor speeds (-100% to 100%) => (-1 to 1)
        public static final double kIntakeSpeed = 1;
        public static final double kReleaseSpeed = -1;

        // Time constants (how much time should the motors spend trying to
        // intake/release the algae).
        public static final double kIntakeTime = 1.0;
        public static final double kReleaseTime = 1.0;

        // Setpoints for tilt motor
        public static final double kTargetGroundIntakeAngle = 130; // The tilt motor's set point before grabbing the
                                                                   // algae
        public static final double kTargetIntakeAngle = 115; // The tilt motor's set point before grabbing the algae
                                                             // out
        // of the reef.
        public static final double kTargetDriveAngle = 0; // The tilt motor's set point when driving around, preferably
                                                          // level.
        public static final double kTargetScoreAngle = 90; // The tilt motor's set point when releasing the algae
                                                           // into
                                                           // the barge.
        public static final double kTargetDisabledAngle = 0; // The tilt motor's set point when it is not being used.
    }

    public static final class CoralMechanismConstants {
        public static final double kCoralKp = .0175;
        public static final double kCoralKi = 0;
        public static final double kCoralKd = 0.00145;

        public static final int kCoralIntakeID = 18;
        public static final int kCoralPivotID = 20;

        public static final double kAllowedSetpointError = 3;

        public static final double kTargetAngleIntake = -160;
        public static final double kTargetAngleStore = -80;
        public static final double kTargetAngleL1 = -59; // down
        public static final double kTargetAngleL2_3 = -58; // left
        public static final double kTargetAngleL4 = -45; // up

    }
}
