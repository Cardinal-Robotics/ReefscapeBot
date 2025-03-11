// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants.CoralMechanismConstants;
import frc.robot.RobotContainer.InteractionState;
import frc.robot.RobotContainer;
import frc.robot.Robot;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax m_pivotMotor = new SparkMax(CoralMechanismConstants.kCoralPivotID, MotorType.kBrushless);

    private final WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(CoralMechanismConstants.kCoralIntakeID);
    private final RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();
    private final ElevatorSubsystem m_elevator;

    private double m_desiredTarget = CoralMechanismConstants.kCoralStore;
    private double m_safetyTarget = CoralMechanismConstants.kCoralStore;

    // Simulation code
    private SingleJointedArmSim m_armSim;
    private SparkMaxSim m_pivotMotorSim;

    public CoralSubsystem(ElevatorSubsystem elevatorSubsystem) {
        m_elevator = elevatorSubsystem;

        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.encoder.positionConversionFactor(Robot.isReal() ? 30 : 60); // Converts rotations into
        // degrees.
        pivotConfig.closedLoop.pid(CoralMechanismConstants.kCoralKp,
                CoralMechanismConstants.kCoralKi, CoralMechanismConstants.kCoralKd)
                .outputRange(-.1, .1);

        m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("CoralTilt", 0);

        if (!Robot.isSimulation())
            return;

        m_pivotMotorSim = new SparkMaxSim(m_pivotMotor, DCMotor.getNEO(1));
        m_armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                15,
                SingleJointedArmSim.estimateMOI(0.5, 10),
                0.5,
                Math.toRadians(-360),
                Math.toRadians(0),
                false,
                0, 0, 0);
    }

    public double getAngle() {
        return m_pivotEncoder.getPosition();
    }

    public Command setMotors(double speed) {
        return run(() -> spinIntakeMotor(speed));
    }

    public void spinIntakeMotor(double speed) {
        m_intakeMotor.set(speed);
    }

    public boolean atTarget() {
        return Math.abs(m_desiredTarget - getAngle()) > CoralMechanismConstants.kAllowedSetpointError;
    }

    public void setTarget(double target) {
        SmartDashboard.putNumber("CoralTilt", target);
        m_desiredTarget = target;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CoralSubsystem::getAngle", getAngle());

        if (m_elevator.getPosition() < m_elevator.getTarget() + .1
                && m_elevator.getPosition() > m_elevator.getTarget() - .1)
            m_safetyTarget = m_desiredTarget;
        else
            m_safetyTarget = CoralMechanismConstants.kCoralStore;

        m_safetyTarget = RobotContainer.interactionState == InteractionState.Coral ? m_safetyTarget
                : CoralMechanismConstants.kCoralStore;

        double currentAngle = getAngle();
        double feedforward = 0.07 * Math.cos(Math.toRadians(currentAngle + 90));

        m_pivotMotor.getClosedLoopController().setReference(
                Robot.isSimulation() ? Math.toRadians(m_safetyTarget) : m_safetyTarget,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                Robot.isSimulation() ? -1 * feedforward : feedforward, ArbFFUnits.kPercentOut);
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        m_armSim.setInput(m_pivotMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_pivotMotorSim.iterate(m_armSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.020);
        m_armSim.update(0.020);
        SmartDashboard.putNumber("CoralSubsystem::getAngle", Math.toDegrees(m_armSim.getAngleRads()));
        setTarget(SmartDashboard.getNumber("CoralTilt", 0));

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }
}
