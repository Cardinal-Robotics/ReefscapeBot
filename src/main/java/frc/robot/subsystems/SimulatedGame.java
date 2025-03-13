// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.dyn4j.world.ContactCollisionData;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;

import org.littletonrobotics.junction.Logger;

public class SimulatedGame extends SubsystemBase {
    private Pose2d[] m_coralSupplyPositions = new Pose2d[] {
            new Pose2d(0.7, 0.6, Rotation2d.kZero),
            new Pose2d(0.7, 7.4, Rotation2d.kZero),
            new Pose2d(16.7, 0.6, Rotation2d.kZero),
            new Pose2d(16.7, 7.4, Rotation2d.kZero)
    };
    private ReefscapeCoralOnField[] m_coralSupplies = new ReefscapeCoralOnField[] {
            new ReefscapeCoralOnField(m_coralSupplyPositions[0]),
            new ReefscapeCoralOnField(m_coralSupplyPositions[1]),
            new ReefscapeCoralOnField(m_coralSupplyPositions[2]),
            new ReefscapeCoralOnField(m_coralSupplyPositions[3])
    };

    private final SwerveDriveSimulation m_swerveSimulation;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final AlgaeSubsystem m_algaeSubsystem;
    private final CoralSubsystem m_coralSubsystem;
    private final SwerveSubsystem m_swerve;

    private IntakeSimulation m_algaeIntakeSim;
    private IntakeSimulation m_coralIntakeSim;

    public SimulatedGame(ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem,
            CoralSubsystem coralSubsystem, SwerveSubsystem swerveSubsystem) {
        m_swerveSimulation = swerveSubsystem.getLibSwerveDrive().getMapleSimDrive().get();
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

        m_algaeIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Algae",
                m_swerveSimulation,
                Meters.of(0.462600),
                Meters.of(0.321503),
                IntakeSide.FRONT,
                1);

        SimulatedArena arena = SimulatedArena.getInstance();
        arena.placeGamePiecesOnField();
        for (ReefscapeCoralOnField coral : m_coralSupplies) {

            arena.addGamePiece(coral);
        }
    }

    @Override
    public void simulationPeriodic() {
        for (int i = 0; i < m_coralSupplies.length; i++) {
            if (i == 0)
                System.out.println(m_coralSupplies[i].getPose3d().getX());
            if (m_coralSupplies[i].isEnabled())
                continue;

            m_coralSupplies[i] = new ReefscapeCoralOnField(m_coralSupplyPositions[i]);
            SimulatedArena.getInstance().addGamePiece(m_coralSupplies[i]);
        }

        Pose3d[] elevatorPoses = m_elevatorSubsystem.getSimulatedElevatorPositions();

        Logger.recordOutput("ZeroedComponentPoses",
                new Pose3d[] {
                        elevatorPoses[0],
                        elevatorPoses[1],
                        new Pose3d(0.235, 0, elevatorPoses[1].getZ() + 0.185,
                                new Rotation3d(0, Math.toRadians(m_algaeSubsystem.getAngle()), 0)),
                        new Pose3d(0.3, 0, elevatorPoses[1].getZ() + 0.365,
                                new Rotation3d(0, Math.toRadians(-m_coralSubsystem.getAngle()), 0))
                });

        handleMechanismIntake(elevatorPoses);
        handleGamePieces(elevatorPoses);
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

        Logger.recordOutput("FieldSimulation/Algae", algaeGamePieces.toArray(Pose3d[]::new));
        Logger.recordOutput("FieldSimulation/Coral", coralGamePieces.toArray(Pose3d[]::new));
    }

    private Transform3d calculateStoredCoralPosition(Pose3d[] elevatorPoses) {
        Transform3d coralPosition = new Transform3d(
                0.3 + (Math.cos(Math.toRadians(90 + m_coralSubsystem.getAngle())) * (0.493332 + 0.0)),
                0,
                elevatorPoses[1].getZ() + 0.365
                        + (Math.sin(Math.toRadians(90 + m_coralSubsystem.getAngle())) * (0.493332 + 0.0)),
                new Rotation3d(0, Math.toRadians(-m_coralSubsystem.getAngle() - 36), 0));
        Transform3d coralOffset = new Transform3d(-0.4, -0.01875, -0.05, Rotation3d.kZero);
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
                MetersPerSecond.of(5),
                Degrees.of(-(-m_coralSubsystem.getAngle() - 36))));
    }
}
