// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.solutions;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.util.Units;

import frc.robot.visual.ArmVisual;

public class ArmMechanism extends SubsystemBase {
    private final SparkMax m_pivotMotor = new SparkMax(1, MotorType.kBrushless);
    private final SparkMaxSim m_pivotMotorSim = new SparkMaxSim(m_pivotMotor, DCMotor.getNEO(1));
    private SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            15,
            SingleJointedArmSim.estimateMOI(0.5, 10),
            0.5,
            Math.toRadians(-360 * 5),
            Math.toRadians(360 * 5),
            false,
            0, 0, 0);

    private ArmVisual visual = new ArmVisual(m_armSim);

    public ArmMechanism() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
                .pid(1, 0.00001, 0.05);

        m_pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Target Angle", 0);
    }

    @Override
    public void periodic() {
        double angle = SmartDashboard.getNumber("Target Angle", 0);
        m_pivotMotor.getClosedLoopController().setReference(angle / 360.0, ControlType.kPosition);
    }

    @Override
    public void simulationPeriodic() {
        m_armSim.setInput(m_pivotMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_pivotMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()), // VERY IMPORTANT. Don't
                                                                                             // forget to convert units.
                RoboRioSim.getVInVoltage(),
                0.020);
        m_armSim.update(0.020);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }
}
