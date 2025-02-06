// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.thethriftybot.ThriftyNova.CurrentType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CoralMechanismConstants;

public class CoralSubsystem extends SubsystemBase {
    final private WPI_TalonSRX m_talon = new WPI_TalonSRX(CoralMechanismConstants.kCoralIntakeID);
    final private SparkMax m_SparkMax = new SparkMax(CoralMechanismConstants.kCoralPivotID, MotorType.kBrushless);
    final private PIDController m_Controller = new PIDController(CoralMechanismConstants.kCoralKp,
            CoralMechanismConstants.kCoralKi, CoralMechanismConstants.kCoralKd);
    final private Encoder m_encoder = new Encoder(CoralMechanismConstants.kCoralEncoderChannelA,
            CoralMechanismConstants.kCoralEncoderChannelB);

    public CoralSubsystem() {
        m_encoder.setDistancePerPulse(4096);
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

    public double setTarget() {
        if (RobotContainer.m_operatorController.povDown().getAsBoolean()) {
            return CoralMechanismConstants.kL1Position;
        } else if (RobotContainer.m_operatorController.povLeft().getAsBoolean()) {
            return CoralMechanismConstants.k2_3Position;
        } else if (RobotContainer.m_operatorController.povUp().getAsBoolean()) {
            return CoralMechanismConstants.kL4Position;
        } else if (RobotContainer.m_operatorController.povRight().getAsBoolean()) {
            return CoralMechanismConstants.kIntakePosition;
        }
        return CoralMechanismConstants.kCoralStore;
    }

    public void alignCoral(double targetAngle) {
        double currentRevs = m_encoder.getDistance();

        m_Controller.setSetpoint(targetAngle);

        double currentAngle = currentRevs * 360;

        double output = m_Controller.calculate(currentAngle);

        m_SparkMax.set(output);
    }

    @Override
    public void periodic() {
    }

}
