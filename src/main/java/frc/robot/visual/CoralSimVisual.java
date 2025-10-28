// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visual;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.Robot;
import frc.robot.Util;

public class CoralSimVisual extends SubsystemBase {
    private Optional<SparkMaxSim> m_simulatedPivot;
    private boolean m_scoringTriggered = false;
    private SparkMaxSim m_simulatedFlywheel;
    private Optional<SparkMax> m_pivot;
    private SparkMax m_flywheel;

    public CoralSimVisual(SparkMaxSim simulatedFlywheel, SparkMax flywheel, SparkMaxSim simulatedPivot,
            SparkMax pivot) {
        m_simulatedFlywheel = simulatedFlywheel;
        m_flywheel = flywheel;

        m_simulatedPivot = Optional.of(simulatedPivot);
        m_pivot = Optional.of(pivot);
        Logger.recordOutput("FieldSimulation/Algae", m_algaeReefPositions);
    }

    public CoralSimVisual(SparkMaxSim simulatedFlywheel, SparkMaxSim simulatedPivot) {
        m_simulatedFlywheel = simulatedFlywheel;
        m_flywheel = Util.getMotorFromSim(simulatedFlywheel);

        m_simulatedPivot = Optional.of(simulatedPivot);
        m_pivot = Optional.of(Util.getMotorFromSim(simulatedPivot));
        Logger.recordOutput("FieldSimulation/Algae", m_algaeReefPositions);
    }

    public CoralSimVisual(SparkMaxSim simulatedFlywheel, SparkMax flywheel) {
        m_simulatedFlywheel = simulatedFlywheel;
        m_flywheel = flywheel;

        m_simulatedPivot = Optional.empty();
        m_pivot = Optional.empty();
        Logger.recordOutput("FieldSimulation/Algae", m_algaeReefPositions);
    }

    public CoralSimVisual(SparkMaxSim simulatedFlywheel) {
        m_simulatedFlywheel = simulatedFlywheel;
        m_flywheel = Util.getMotorFromSim(simulatedFlywheel);

        m_simulatedPivot = Optional.empty();
        m_pivot = Optional.empty();
        Logger.recordOutput("FieldSimulation/Algae", m_algaeReefPositions);
        scoreCoral();
    }

    @Override
    public void simulationPeriodic() {
        double pivotAngle = 30; // m_pivot.isPresent() ? m_pivot.get().getEncoder().getPosition() * 360 : 0

        List<GamePiece> coralGamePieces = SimulatedArena.getInstance().getGamePiecesByType("Coral");
        List<Pose3d> coralPoses = new ArrayList<Pose3d>();
        coralGamePieces.forEach((GamePiece piece) -> coralPoses.add(piece.getPose3d()));

        Pose3d robotPosition = new Pose3d(3.2, 3.87, 0, Rotation3d.kZero);
        Logger.recordOutput("RobotPosition", robotPosition);

        Logger.recordOutput("ZeroedComponentPoses",
                new Pose3d[] {
                        Pose3d.kZero,
                        new Pose3d(0, 0, 0.19, Rotation3d.kZero),
                        new Pose3d(0.235, 0, 0.185 + 0.19, Rotation3d.kZero),
                        new Pose3d(0.3, 0, 0.365 + 0.19,
                                new Rotation3d(0, Math.toRadians(pivotAngle), 0))
                });

        if (!m_scoringTriggered)
            coralPoses.add(robotPosition.plus(calculateStoredCoralPosition()));

        Logger.recordOutput("FieldSimulation/Coral", coralPoses.toArray(Pose3d[]::new));
    }

    private Transform3d calculateStoredCoralPosition() {
        Transform3d coralOffset = new Transform3d(-0.1, -0.015,
                0.355 + 0.17, Rotation3d.kZero);

        return new Transform3d(
                0.3,
                0,
                0.365,
                new Rotation3d(0,
                        Math.toRadians(30), 0)) // m_pivot.isPresent() ? m_pivot.get().getEncoder().getPosition() * 360
                                                // : 0
                .plus(coralOffset);
    }

    private void scoreCoral() {
        Transform3d storedCoralPosition = calculateStoredCoralPosition();
        Pose3d robotPosition = new Pose3d(3.24, 3.87, 0, Rotation3d.kZero);

        SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
                robotPosition.getTranslation().toTranslation2d(),
                new Translation2d(storedCoralPosition.getX(), storedCoralPosition.getY()),
                new ChassisSpeeds(0, 0, 0),
                Rotation2d.kZero,
                Meters.of(storedCoralPosition.getZ()),
                MetersPerSecond.of(0.75),
                Degrees.of(-Math.toDegrees(storedCoralPosition.getRotation().getY()))));
    }

    private Pose3d[] m_algaeReefPositions = new Pose3d[] {
            new Pose3d(4.148855943453357, 3.4334397031146104, 0.909320, Rotation3d.kZero),
            new Pose3d(4.149125005470327, 4.610563326115974, 0.909320, Rotation3d.kZero),
            new Pose3d(5.179124964694157, 4.026436851741331, 0.909320, Rotation3d.kZero),
            new Pose3d(17.548 - 4.148855943453357, 8.052 - 3.4334397031146104, 0.909320, Rotation3d.kZero),
            new Pose3d(17.548 - 4.149125005470327, 8.052 - 4.610563326115974, 0.909320, Rotation3d.kZero),
            new Pose3d(17.548 - 5.179124964694157, 8.052 - 4.026436851741331, 0.909320, Rotation3d.kZero),
            new Pose3d(4.832333594185316, 3.438253785007526, 1.313180, Rotation3d.kZero),
            new Pose3d(3.7987872165986545, 4.017257939318807, 1.313180, Rotation3d.kZero),
            new Pose3d(4.836114123343362, 4.631850049426585, 1.313180, Rotation3d.kZero),

            new Pose3d(17.548 - 4.832333594185316, 8.052 - 3.438253785007526, 1.313180, Rotation3d.kZero),
            new Pose3d(17.548 - 3.7987872165986545, 8.052 - 4.017257939318807, 1.313180, Rotation3d.kZero),
            new Pose3d(17.548 - 4.836114123343362, 8.052 - 4.631850049426585, 1.313180, Rotation3d.kZero),
    };
}
