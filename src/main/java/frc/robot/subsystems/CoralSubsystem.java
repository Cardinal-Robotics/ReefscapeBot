// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.thethriftybot.ThriftyNova.CurrentType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CoralMechanismConstants;

public class CoralSubsystem extends SubsystemBase {
    final private WPI_TalonSRX m_talon = new WPI_TalonSRX(CoralMechanismConstants.kCoralIntakeID);
    final private SparkMax m_SparkMax = new SparkMax(CoralMechanismConstants.kCoralPivotID, MotorType.kBrushless);
    final private PIDController m_Controller = new PIDController(CoralMechanismConstants.kCoralKp,
            CoralMechanismConstants.kCoralKi, CoralMechanismConstants.kCoralKd);
    final private DutyCycleEncoder m_encoder = new DutyCycleEncoder(CoralMechanismConstants.kCoralEncoderChannelA);

    boolean L1 = false;
    boolean L2_3 = false;
    boolean L4 = false;
    boolean Intake = false;
    boolean CoralStore = true;

    public CoralSubsystem() {

    }

    public double getRot() {
        return m_encoder.get();
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
        double pos;
        if (RobotContainer.m_operatorController.povDown().getAsBoolean()) {
            L1 = true;
            L2_3 = false;
            L4 = false;
            Intake = false;
            CoralStore = false;
        } else if (RobotContainer.m_operatorController.povLeft().getAsBoolean()) {
            L1 = false;
            L2_3 = true;
            L4 = false;
            Intake = false;
            CoralStore = false;
        } else if (RobotContainer.m_operatorController.povUp().getAsBoolean()) {
            L1 = false;
            L2_3 = false;
            L4 = true;
            Intake = false;
            CoralStore = false;
        } else if (RobotContainer.m_operatorController.povRight().getAsBoolean()) {
            L1 = false;
            L2_3 = false;
            L4 = false;
            Intake = true;
            CoralStore = false;
        } else if (RobotContainer.m_operatorController.y().getAsBoolean()) {
            L1 = false;
            L2_3 = false;
            L4 = false;
            Intake = false;
            CoralStore = true;
        }
        if (L1) {
            return CoralMechanismConstants.kL1Position;
        } else if (L2_3) {
            return CoralMechanismConstants.k2_3Position;
        } else if (L4) {
            return CoralMechanismConstants.kL4Position;
        } else if (Intake) {
            return CoralMechanismConstants.kIntakePosition;
        } else {
            return CoralMechanismConstants.kCoralStore;
        }

    }

    /*
     * public void sigmaCalc() {
     * 
     * m_SparkMax.set(.06);
     * 
     * }
     */

    public void alignCoral(double targetAngle) {
        double currentRevs = getRot();

        m_Controller.setSetpoint(targetAngle);

        double currentAngle = currentRevs * 360 - 156;// - 277

        double feedforward = 0.06 * Math.cos(Math.toRadians(currentAngle));

        double output = m_Controller.calculate(currentAngle);
        SmartDashboard.putNumber("feedforward", feedforward);
        if (output > 0.1) {
            output = 0.1;
        } else if (output < -0.15) {
            output = -0.15;
        }
        SmartDashboard.putNumber("pid output", output);

        m_SparkMax.set(-output + feedforward);
    }

    @Override
    public void periodic() {
    }

}
