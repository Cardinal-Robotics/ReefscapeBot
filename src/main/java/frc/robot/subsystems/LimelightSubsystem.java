// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.util.Units;

import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.simulation.PhotonCameraSim;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Robot;

public class LimelightSubsystem extends SubsystemBase {
    private final SwerveSubsystem m_swerveDrive;

    private final PhotonPoseEstimator m_photonRightPoseEstimator;
    private final PhotonPoseEstimator m_photonLeftPoseEstimator;

    private final PhotonCamera m_rightCamera;
    private final PhotonCamera m_leftCamera;

    StructPublisher<Pose3d> m_publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Photon Pose", Pose3d.struct).publish();

    public LimelightSubsystem(SwerveSubsystem swerveSubsystem) {
        m_swerveDrive = swerveSubsystem;

        m_leftCamera = new PhotonCamera("leftCamera");
        Transform3d robotToLeftCamera = new Transform3d(
                new Translation3d(0.1, 0, 0.5),
                new Rotation3d(0, Math.toRadians(-15), 0));

        m_rightCamera = new PhotonCamera("rightCamera");
        Transform3d robotToRightCamera = new Transform3d(
                new Translation3d(0.1, 0, 0.5),
                new Rotation3d(0, Math.toRadians(-15), 0));

        m_photonRightPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToRightCamera);

        m_photonLeftPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToLeftCamera);
    }

    @Override
    public void periodic() {
        var result = m_leftCamera.getLatestResult();
        if (!result.hasTargets())
            return;

        Optional<EstimatedRobotPose> getEstimatedGlobalPose()
        m_photonLeftPoseEstimator.update(null)
        m_swerveDrive.getLibSwerveDrive().addVisionMeasurement(null, 0);
    }

}
