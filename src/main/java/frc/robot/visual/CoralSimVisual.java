// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visual;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import java.lang.reflect.Field;
import java.util.Optional;

import frc.robot.Robot;
import frc.robot.Util;

public class CoralSimVisual extends SubsystemBase {
    private Optional<SparkMaxSim> m_simulatedPivot;
    private SparkMaxSim m_simulatedFlywheel;
    private Optional<SparkMax> m_pivot;
    private SparkMax m_flywheel;

    public CoralSimVisual(SparkMaxSim simulatedFlywheel, SparkMax flywheel, SparkMaxSim simulatedPivot,
            SparkMax pivot) {
        m_simulatedFlywheel = simulatedFlywheel;
        m_flywheel = flywheel;

        m_simulatedPivot = Optional.of(simulatedPivot);
        m_pivot = Optional.of(pivot);
    }

    public CoralSimVisual(SparkMaxSim simulatedFlywheel, SparkMaxSim simulatedPivot) {
        m_simulatedFlywheel = simulatedFlywheel;
        m_flywheel = Util.getMotorFromSim(simulatedFlywheel);

        m_simulatedPivot = Optional.of(simulatedPivot);
        m_pivot = Optional.of(Util.getMotorFromSim(simulatedPivot));
    }

    public CoralSimVisual(SparkMaxSim simulatedFlywheel, SparkMax flywheel) {
        m_simulatedFlywheel = simulatedFlywheel;
        m_flywheel = flywheel;
    }

    public CoralSimVisual(SparkMaxSim simulatedFlywheel) {
        m_simulatedFlywheel = simulatedFlywheel;
        m_flywheel = Util.getMotorFromSim(simulatedFlywheel);
    }

    @Override
    public void simulationPeriodic() {
        double pivotRadianRot = m_pivot.isEmpty() ? 0 : m_pivot.get().getEncoder().getPosition();

        System.out.println(m_flywheel.get());

        Logger.recordOutput("ZeroedComponentPoses",
                new Pose3d[] {
                        Pose3d.kZero,
                        Pose3d.kZero,
                        new Pose3d(0.235, 0, 0.185, Rotation3d.kZero),
                        new Pose3d(0.3, 0, 0.365,
                                new Rotation3d(0, pivotRadianRot * 360, 0))
                });

        Logger.recordOutput("FieldSimulation/Coral", new Pose3d[] {});
    }
}
