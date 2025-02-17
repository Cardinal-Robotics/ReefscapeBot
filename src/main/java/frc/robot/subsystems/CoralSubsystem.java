// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.CoralMechanismConstants;
import frc.robot.RobotContainer;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax m_pivotMotor = new SparkMax(CoralMechanismConstants.kCoralPivotID, MotorType.kBrushless);
    private final WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(CoralMechanismConstants.kCoralIntakeID);

    private double m_setpoint = 0;

    public CoralSubsystem() {
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.absoluteEncoder.positionConversionFactor(360); // Converts rotations into degrees.
        pivotConfig.closedLoop.pid(CoralMechanismConstants.kCoralKp,
                CoralMechanismConstants.kCoralKi, CoralMechanismConstants.kCoralKd);

        m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double getAngle() {
        return m_pivotMotor.getAbsoluteEncoder().getPosition();
    }

    public void spinIntakeMotor(double speed) {
        m_intakeMotor.set(speed);
    }

    public void setTarget(double target) {
        m_setpoint = target;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral mechanism rotation", getAngle());// - 277

        double currentAngle = getAngle();
        double feedforward = 0.06 * Math.cos(Math.toRadians(currentAngle));

        /*
         * m_pivotMotor.getClosedLoopController().setReference(
         * m_setpoint,
         * ControlType.kPosition,
         * ClosedLoopSlot.kSlot0,
         * feedforward);
         */
    }

}
