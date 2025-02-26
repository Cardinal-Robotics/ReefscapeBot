// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedGame extends SubsystemBase {
    public SimulatedGame() {
        SimulatedArena.getInstance().placeGamePiecesOnField();

        SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(
                // We must specify a heading since the coral is a tube
                new Translation2d(2, 2)));

        SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)));

        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }

    @Override
    public void periodic() {

    }
}
