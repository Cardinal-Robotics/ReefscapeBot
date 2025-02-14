// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

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

    @Override
    public void execute() {
        Optional<Pose2d> potentialPose = m_visionSubsystem.getRobotPoseRelativeToAprilTag(6, Cameras.LEFT_CAM);
        if (potentialPose.isEmpty())
            return;

        Pose2d pose = potentialPose.get();

        m_swerveSubsystem.driveCustomPoseOriented(
                new Pose2d(Translation2d.kZero,
                        Rotation2d.fromDegrees(
                                m_visionSubsystem.getAprilTagPose(6, Transform2d.kZero).getRotation().getRadians())),
                -pose.getY(),
                -pose.getX(), -pose.getRotation().getDegrees());

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
