// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
    final private WPI_TalonSRX m_talon = new WPI_TalonSRX(1);
    final private SparkMax m_SparkMax = new SparkMax(3, MotorType.kBrushless);
    final private PIDController m_Controller = new PIDController(0, 0, 0);
    double targetAngle = Math.toRadians(0);

    public CoralSubsystem() {
        m_Controller.setSetpoint(targetAngle);
        m_Controller.calculate();
    }

    public void takeIn() {
        m_talon.set(-.2);
    }

    public void moveOut() {
        m_talon.set(.2);
    }

    public void stop() {
        m_talon.set(0);
    }

    @Override
    public void periodic() {
    }

}
