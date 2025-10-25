// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.visual.CoralSimVisual;

public class CoralSubsystem extends SubsystemBase {
    SparkMax flywheel = new SparkMax(0, MotorType.kBrushless);
    SparkMaxSim flywheelSim = new SparkMaxSim(flywheel, DCMotor.getNEO(1));

    private CoralSimVisual simVisual = new CoralSimVisual(flywheelSim);

    public CoralSubsystem() {
    }

    @Override
    public void simulationPeriodic() {
        // Use in later part.
        /*
         * m_armSim.setInput(m_pivotMotorSim.getAppliedOutput() *
         * RobotController.getBatteryVoltage());
         * m_pivotMotorSim.iterate(m_armSim.getVelocityRadPerSec(),
         * RoboRioSim.getVInVoltage(), 0.020);
         * m_armSim.update(0.020);
         * 
         * RoboRioSim.setVInVoltage(
         * BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()
         * ));
         */
    }
}
