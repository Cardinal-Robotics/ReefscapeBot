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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import frc.robot.Robot;

public class LimelightSubsystem extends SubsystemBase {
    private final SwerveSubsystem m_swerveDrive;
    private NetworkTable m_table;

    StructPublisher<Pose3d> m_publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Photon Pose", Pose3d.struct).publish();

    // Simulation code
    VisionSystemSim m_visionSimulation;

    public LimelightSubsystem(SwerveSubsystem swerveSubsystem) {
        m_table = NetworkTableInstance.getDefault().getTable("limelight"); // gets 8 values from limelight

        m_swerveDrive = swerveSubsystem;

        if (!Robot.isSimulation())
            return;

        m_visionSimulation = new VisionSystemSim("main");
        m_visionSimulation.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)); // If this
                                                                                                       // complains that
                                                                                                       // k2025Reefscape
                                                                                                       // doesn't exist,
                                                                                                       // you need to
                                                                                                       // update WPILib
                                                                                                       // tools.
                                                                                                       // https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCamera camera = new PhotonCamera("cameraName");
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        m_visionSimulation.addCamera(cameraSim, robotToCamera);

    }

    public boolean hasTargets() {
        return m_table.getEntry("tv").getInteger(0) == 1;
    }

    public Pose2d getTagPoseRelativeToRobot() {
        double[] poseData = m_table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

        Pose2d pose = new Pose2d(new Translation2d(poseData[0], poseData[1]),
                new Rotation2d(Units.degreesToRadians(poseData[4])));

        m_publisher.set(new Pose3d(pose));

        return pose;
    }

    public double getYaw() { // gets the yaw
        return m_table.getEntry("botpose").getDoubleArray(new double[6])[5] - 90;
    }

    public double getRedX() {
        return m_table.getEntry("botpose_wpired").getDoubleArray(new double[6])[0];
    }

    public double getRedY() {
        return m_table.getEntry("botpose_wpired").getDoubleArray(new double[6])[1];
    }

    public double alignYaw() {
        double omega = 0; // angular velocity
        if (getYaw() < -92) {
            omega = .5;
        } else if (getYaw() > 88) {
            omega = -.5;
        }

        return omega; // returns omega
    }

    @Override
    public void periodic() {
        if (!Robot.isSimulation())
            return;

        m_table = NetworkTableInstance.getDefault().getTable("limelight");
        m_visionSimulation.update(m_swerveDrive.getLibSwerveDrive().getPose());

        VisionTargetSim[] targets = m_visionSimulation.getVisionTargets().toArray(new VisionTargetSim[0]);
        for (VisionTargetSim target : targets) {
            if (target.fiducialID != 6)
                continue;

            Pose3d relativePose = target.getPose().relativeTo(new Pose3d(m_swerveDrive.getPose()));

            // m_table.getEntry("botpose_targetspace").setDoubleArray(relativePose);

            m_publisher.set(relativePose);
            break;
        }
    }

}
