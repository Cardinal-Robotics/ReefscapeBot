// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.dyn4j.geometry.MassType;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;

import org.littletonrobotics.junction.Logger;

public class SimulatedGame extends SubsystemBase {
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

    private Pose2d[] m_coralSupplyPositions = new Pose2d[] {
            new Pose2d(0.7, 0.6, Rotation2d.kZero),
            new Pose2d(0.7, 7.4, Rotation2d.kZero),
            new Pose2d(16.7, 0.6, Rotation2d.kZero),
            new Pose2d(16.7, 7.4, Rotation2d.kZero)
    };

    private ReefscapeCoralOnField[] m_coralSupplies = new ReefscapeCoralOnField[] {
            null,
            null,
            null,
            null
    };

    private final SwerveDriveSimulation m_swerveSimulation;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final AlgaeSubsystem m_algaeSubsystem;
    private final CoralSubsystem m_coralSubsystem;
    private final SwerveSubsystem m_swerve;

    private IntakeSimulation m_algaeIntakeSim;
    private IntakeSimulation m_coralIntakeSim;
    private SimulatedArena m_arena;

    public SimulatedGame(ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem,
            CoralSubsystem coralSubsystem, SwerveSubsystem swerveSubsystem) {
        m_swerveSimulation = swerveSubsystem.getLibSwerveDrive().getMapleSimDrive().orElse(null);
        m_elevatorSubsystem = elevatorSubsystem;
        m_algaeSubsystem = algaeSubsystem;
        m_coralSubsystem = coralSubsystem;
        m_swerve = swerveSubsystem;

        if (!Robot.isSimulation())
            return;

        m_coralIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Coral",
                m_swerveSimulation,
                Meters.of(0.142976),
                Meters.of(0.493332),
                IntakeSide.FRONT,
                1);
        m_coralIntakeSim.addGamePieceToIntake();

        m_algaeIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Algae",
                m_swerveSimulation,
                Meters.of(0.462600),
                Meters.of(0.321503),
                IntakeSide.FRONT,
                1);

        m_arena = SimulatedArena.getInstance();
        m_arena.placeGamePiecesOnField();

        for (int i = 0; i < m_coralSupplies.length; i++) {
            if (m_coralSupplies[i] != null)
                continue;

            m_coralSupplies[i] = new ReefscapeCoralOnField(m_coralSupplyPositions[i]);
            SimulatedArena.getInstance().addGamePiece(m_coralSupplies[i]);
        }

        for (Pose3d pose : m_algaeReefPositions) {
            createReefAlgae(pose);
        }
    }

    @Override
    public void simulationPeriodic() {
        Pose3d[] elevatorPoses = m_elevatorSubsystem.getSimulatedElevatorPositions();

        Logger.recordOutput("ZeroedComponentPoses",
                new Pose3d[] {
                        elevatorPoses[0],
                        elevatorPoses[1],
                        new Pose3d(0.235, 0, elevatorPoses[1].getZ() + 0.185,
                                new Rotation3d(0, Math.toRadians(m_algaeSubsystem.getAngle()), 0)),
                        new Pose3d(0.3, 0, elevatorPoses[1].getZ() + 0.365,
                                new Rotation3d(0, Math.toRadians(m_coralSubsystem.getAngle()), 0))
                });

        handleMechanismIntake(elevatorPoses);
        handleGamePieces(elevatorPoses);
    }

    private void createReefAlgae(Pose3d pose) {
        GamePieceOnFieldSimulation gamePiece = new GamePieceOnFieldSimulation(
                ReefscapeAlgaeOnField.REEFSCAPE_ALGAE_INFO, () -> pose.getZ(),
                new Pose2d(pose.getX(), pose.getY(), Rotation2d.kZero), Translation2d.kZero);
        gamePiece.setMass(MassType.INFINITE);

        m_arena.addGamePiece(gamePiece);
    }

    private void handleMechanismIntake(Pose3d[] elevatorPoses) {
        double algaeIntakeSpeed = m_algaeSubsystem.getIntakeSpeed();
        double coralIntakeSpeed = m_coralSubsystem.getIntakeSpeed();

        if (algaeIntakeSpeed == 0)
            m_algaeIntakeSim.stopIntake();
        else if (algaeIntakeSpeed < 0)
            m_algaeIntakeSim.startIntake();
        else if (algaeIntakeSpeed > 0 && m_algaeIntakeSim.getGamePiecesAmount() == 1)
            scoreAlgae();

        if (coralIntakeSpeed == 0)
            m_coralIntakeSim.stopIntake();
        else if (coralIntakeSpeed > 0)
            m_coralIntakeSim.startIntake();
        else if (coralIntakeSpeed < 0 && m_coralIntakeSim.getGamePiecesAmount() == 1)
            scoreCoral(elevatorPoses);
    }

    // No, you won't be getting a single explanation on how this works. I do not
    // ever want to look at this code again.
    private void handleGamePieces(Pose3d[] elevatorPoses) {
        List<Pose3d> algaeGamePieces = SimulatedArena.getInstance().getGamePiecesByType("Algae");
        List<Pose3d> coralGamePieces = SimulatedArena.getInstance().getGamePiecesByType("Coral");

        if (m_algaeIntakeSim.getGamePiecesAmount() == 1) {
            algaeGamePieces.add(
                    new Pose3d(m_swerve.getPose())
                            .plus(new Transform3d(
                                    0.235 + (Math.cos(Math.toRadians(90 - m_algaeSubsystem.getAngle()))
                                            * (0.321503 + 0.07)),
                                    0,
                                    elevatorPoses[1].getZ() + 0.185
                                            + (Math.sin(Math.toRadians(90 - m_algaeSubsystem.getAngle()))
                                                    * (0.321503 + 0.07)),
                                    Rotation3d.kZero)));
        }
        if (m_coralIntakeSim.getGamePiecesAmount() == 1) {
            coralGamePieces.add(new Pose3d(m_swerve.getPose()).plus(calculateStoredCoralPosition(elevatorPoses)));
        }

        for (int i = 0; i < m_coralSupplies.length; i++) {
            m_coralSupplies[i].setTransform(GeometryConvertor.toDyn4jTransform(m_coralSupplyPositions[i]));
            if (m_coralSupplies[i].getOwner() != null)
                continue;

            m_coralSupplies[i] = new ReefscapeCoralOnField(m_coralSupplyPositions[i]);
            m_arena.addGamePiece(m_coralSupplies[i]);

        }

        Logger.recordOutput("FieldSimulation/Algae", algaeGamePieces.toArray(Pose3d[]::new));
        Logger.recordOutput("FieldSimulation/Coral", coralGamePieces.toArray(Pose3d[]::new));
    }

    private Transform3d calculateStoredCoralPosition(Pose3d[] elevatorPoses) {
        Transform3d coralPosition = new Transform3d(
                0.3,
                0,
                elevatorPoses[1].getZ() + 0.365,
                new Rotation3d(0, Math.toRadians(m_coralSubsystem.getAngle()), 0));

        Transform3d coralOffset = new Transform3d(-0.1, -0.015,
                0.355, Rotation3d.kZero);

        return coralPosition.plus(coralOffset);
    }

    private void scoreAlgae() {
        m_algaeIntakeSim.obtainGamePieceFromIntake();
        SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                m_swerveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                new Translation2d(0.5, 0),
                m_swerveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                m_swerveSimulation.getSimulatedDriveTrainPose().getRotation(),
                Meters.of(m_elevatorSubsystem.getPosition() + 0.185),
                MetersPerSecond.of(2),
                Degrees.of(90 - m_algaeSubsystem.getAngle())));
    }

    private void scoreCoral(Pose3d[] elevatorPoses) {
        m_coralIntakeSim.obtainGamePieceFromIntake();
        Transform3d storedCoralPosition = calculateStoredCoralPosition(elevatorPoses);

        SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
                m_swerveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                new Translation2d(storedCoralPosition.getX(), storedCoralPosition.getY()),
                m_swerveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                m_swerveSimulation.getSimulatedDriveTrainPose().getRotation(),
                Meters.of(storedCoralPosition.getZ()),
                MetersPerSecond.of(1.5),
                Degrees.of(-Math.toDegrees(storedCoralPosition.getRotation().getY()))));
    }
}
