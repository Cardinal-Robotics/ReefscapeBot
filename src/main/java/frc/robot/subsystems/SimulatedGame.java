// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.littletonrobotics.junction.Logger;

public class SimulatedGame extends SubsystemBase {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;
    private final AlgaeSubsystem m_algaeSubsystem;
    private final CoralSubsystem m_coralSubsystem;

    private IntakeSimulation m_coralIntakeSim;

    public SimulatedGame(ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem,
            CoralSubsystem coralSubsystem, SwerveSubsystem swerveSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_swerveSubsystem = swerveSubsystem;
        m_algaeSubsystem = algaeSubsystem;
        m_coralSubsystem = coralSubsystem;

        if (!Robot.isSimulation())
            return;

        /*
         * m_coralIntakeSim = IntakeSimulation.OverTheBumperIntake(
         * "Coral",
         * m_swerveSubsystem.getLibSwerveDrive().getMapleSimDrive().get(),
         * Meters.of(0.142976),
         * Meters.of(0.493332),
         * IntakeSide.FRONT,
         * 1);
         */

        SimulatedArena.getInstance().placeGamePiecesOnField();

    }

    @Override
    public void simulationPeriodic() {
        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

        Pose3d[] elevatorPoses = m_elevatorSubsystem.getSimulatedElevatorPositions();

        Logger.recordOutput("ZeroedComponentPoses",
                new Pose3d[] {
                        elevatorPoses[0],
                        elevatorPoses[1],
                        new Pose3d(0.235, 0, elevatorPoses[1].getZ() + 0.185,
                                new Rotation3d(0, m_algaeSubsystem.getAngle(), 0)),
                        new Pose3d(0.3, 0, elevatorPoses[1].getZ() + 0.365,
                                new Rotation3d(0, -m_coralSubsystem.getAngle(), 0))
                });
    }
}
