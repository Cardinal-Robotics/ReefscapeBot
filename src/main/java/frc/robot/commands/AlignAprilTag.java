// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.VisionSubsystem.Cameras;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignAprilTag extends Command {
    private final VisionSubsystem m_visionSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    public AlignAprilTag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        m_visionSubsystem = visionSubsystem;
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(m_visionSubsystem);
    }

    public enum TagPositions {
        TOP_RIGHT,
        TOP_LEFT,
        TOP,

        BOTTOM_RIGHT,
        BOTTOM_LEFT,
        BOTTOM,
    }

    private TagPositions m_tagPosition = TagPositions.TOP;

    public void setTagPosition(TagPositions tagPosition) {
        m_tagPosition = tagPosition;
    }

    @Override
    public void execute() {
        int ID = 10;
        switch (m_tagPosition) {
            case TOP:
                ID = 10;
                break;
            case TOP_RIGHT:
                ID = 9;
                break;
            case TOP_LEFT:
                ID = 11;
                break;
            case BOTTOM:
                ID = 7;
                break;
            case BOTTOM_RIGHT:
                ID = 8;
                break;
            case BOTTOM_LEFT:
                ID = 6;
                break;

        }

        Optional<Pose2d> potentialPose = Optional
                .of(m_swerveSubsystem.getPose().relativeTo(VisionSubsystem.getAprilTagPose(ID, Transform2d.kZero)));

        if (potentialPose.isEmpty())
            return;

        Pose2d pose = potentialPose.get();

        m_swerveSubsystem.driveCustomPoseOriented(
                new Pose2d(Translation2d.kZero,
                        Rotation2d.fromDegrees(
                                m_visionSubsystem.getAprilTagPose(ID, Transform2d.kZero).getRotation().getDegrees())),
                -pose.getX(),
                -pose.getY(), pose.getRotation().getDegrees());

        // this aligns first with x then goes with y once x aligned
        /*
         * if (y > 2) { // moves robot right
         * m_swerveSubsystem.driveRelative(0, 0.8, 0);
         * } else if (y < -2) { // left
         * m_swerveSubsystem.driveRelative(0, -0.8, 0);
         * }
         * 
         * // If x and y aren't aligned yet, don't align the closeness yet.
         * if (!(y < 2 && y > -2))
         * return;
         * 
         * if (x < 0.5) { // back away from the AprilTag
         * m_swerveSubsystem.driveRelative(-0.8, 0, 0);
         * } else if (x > 0.6) { // move to the AprilTag
         * m_swerveSubsystem.driveRelative(0.8, 0, 0);
         * }
         */

    }
}
