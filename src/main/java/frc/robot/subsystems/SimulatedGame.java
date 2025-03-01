// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class SimulatedGame extends SubsystemBase {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final AlgaeSubsystem m_algaeSubsystem;

    public SimulatedGame(ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_algaeSubsystem = algaeSubsystem;

        if (!Robot.isSimulation())
            return;

        SimulatedArena.getInstance().placeGamePiecesOnField();

        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }

    @Override
    public void simulationPeriodic() {
        Pose3d[] elevatorPoses = m_elevatorSubsystem.getSimulatedElevatorPositions();

        Logger.recordOutput("ZeroedComponentPoses",
                new Pose3d[] {
                        elevatorPoses[0],
                        elevatorPoses[1],
                        new Pose3d(0.235, 0, elevatorPoses[1].getZ() + 0.185,
                                new Rotation3d(0, m_algaeSubsystem.getAngle(), 0)),
                        new Pose3d(0, 0, elevatorPoses[1].getZ(), Rotation3d.kZero)
                });
    }
}
