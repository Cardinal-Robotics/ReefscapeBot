// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visual;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;

public class GenericMotorVisual extends SubsystemBase {
    private SparkMaxSim m_motorSim;
    private SparkMax m_motor;
    private double m_angle;

    private final Mechanism2d mech = new Mechanism2d(3, 3);
    private final MechanismRoot2d root = mech.getRoot("Motor", 1.5, 1.5);
    private final MechanismLigament2d wheel = new MechanismLigament2d("Wheel", 1.0, 0);

    private final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
            0.00096,
            1.0);
    private final FlywheelSim flywheelSim = new FlywheelSim(flywheelPlant, DCMotor.getNEO(1));

    public GenericMotorVisual(SparkMaxSim motorSim, SparkMax motor) {
        m_motorSim = motorSim;
        m_motor = motor;
        initialize();
    }

    public GenericMotorVisual(SparkMaxSim motorSim) {
        m_motorSim = motorSim;
        m_motor = Util.getMotorFromSim(motorSim);
        initialize();
    }

    public void initialize() {
        root.append(wheel);
        SmartDashboard.putData("Motor Visualization", mech);
    }

    @Override
    public void simulationPeriodic() {
        double voltage = m_motor.get() * RobotController.getBatteryVoltage();
        double dt = 0.02;

        // Apply input voltage
        flywheelSim.setInputVoltage(voltage);
        flywheelSim.update(dt);

        m_motorSim.iterate(flywheelSim.getAngularVelocityRadPerSec(), voltage, dt);

        m_angle += flywheelSim.getAngularVelocityRadPerSec() * dt;
        wheel.setAngle(Math.toDegrees(m_angle));

        // Display info
        SmartDashboard.putNumber("Flywheel Speed (RPM)", flywheelSim.getAngularVelocityRPM());
        SmartDashboard.putNumber("Flywheel Angle (deg)", Math.toDegrees(m_angle));
    }
}
