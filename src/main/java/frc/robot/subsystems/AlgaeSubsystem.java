// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.AlgaeMechanismConstants;
import frc.robot.Constants.CoralMechanismConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorTarget;
import frc.robot.RobotContainer.InteractionState;
import frc.robot.RobotContainer;
import frc.robot.Robot;

public class AlgaeSubsystem extends SubsystemBase {
    private final RelativeEncoder m_tiltEncoder;
    private final ElevatorSubsystem m_elevator;
    private final ArmFeedforward m_feedforward;
    private final WPI_TalonSRX m_intakeMotor;
    private final SparkMax m_tiltMotor;

    private double m_desiredTarget;
    private double m_realTarget;

    // Simulation code
    private SingleJointedArmSim m_armSim;
    private SparkMaxSim m_tiltMotorSim;

    public AlgaeSubsystem(ElevatorSubsystem elevator) {
        SmartDashboard.putNumber("AlgaeTilt", 0);
        m_intakeMotor = new WPI_TalonSRX(AlgaeMechanismConstants.kIntakeMotorPort);
        m_tiltMotor = new SparkMax(AlgaeMechanismConstants.kTiltMotorPort, MotorType.kBrushless);
        m_feedforward = new ArmFeedforward(0.1, 0.1, 0.1);
        m_elevator = elevator;

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);

        SparkMaxConfig tiltConfig = new SparkMaxConfig();
        tiltConfig.idleMode(IdleMode.kBrake);
        tiltConfig.closedLoop.pid(
                AlgaeMechanismConstants.kTiltKp,
                AlgaeMechanismConstants.kTiltKi,
                AlgaeMechanismConstants.kTiltKd)
                .outputRange(-0.2, 0.1);

        // Simulation only gives radians, so I have to do this
        tiltConfig.encoder.positionConversionFactor(30);

        m_tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_tiltEncoder = m_tiltMotor.getEncoder();

        if (!Robot.isSimulation())
            return;

        m_tiltMotorSim = new SparkMaxSim(m_tiltMotor, DCMotor.getNEO(1));
        m_armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                15,
                SingleJointedArmSim.estimateMOI(0.5, 10),
                0.5,
                0,
                Math.toRadians(180),
                false,
                0, 0, 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaeSubsystem::getPosition", getAngle());
        SmartDashboard.putNumber("algae motor %", m_tiltMotor.getAppliedOutput());
        // m_desiredTarget = SmartDashboard.getNumber("AlgaeTilt", getAngle());

        m_realTarget = (RobotContainer.interactionState == InteractionState.Algae) ? m_desiredTarget
                : AlgaeMechanismConstants.kTargetDisabledAngle;

        m_realTarget = (m_elevator.getPosition() >= ElevatorTarget.AlgaeScore.getAlgaePosition() - 0.1) ? m_realTarget
                : AlgaeMechanismConstants.kTargetDisabledAngle;

        double currentAngle = getAngle();
        double feedforward = 0.04 * Math.cos(Math.toRadians(currentAngle - 90));

        m_tiltMotor.getClosedLoopController().setReference(
                m_realTarget,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward, ArbFFUnits.kPercentOut);
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        m_armSim.setInput(m_tiltMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_tiltMotorSim.iterate(m_armSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.020);
        m_armSim.update(0.020);
        SmartDashboard.putNumber("AlgaeSubsystem::getPosition", Math.toDegrees(m_armSim.getAngleRads()));

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }

    /**
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void spinIntakeMotor(double speed) {
        m_intakeMotor.set(speed);
    }

    public Command setMotors(double speed) {
        return run(() -> spinIntakeMotor(speed));
    }

    public void stopIntakeMotor() {
        spinIntakeMotor(0);
    }

    public double getAngle() {
        if (Robot.isSimulation())
            return m_armSim.getAngleRads();

        return m_tiltEncoder.getPosition();
    }

    public void setTiltTarget(double setpoint) {
        m_desiredTarget = Robot.isSimulation() ? Math.toRadians(setpoint) : setpoint;
        SmartDashboard.putNumber("AlgaeTarget", m_desiredTarget);
    }

    public boolean isTiltMotorAtGoal(double target) {
        double currentPosition = getAngle();
        double error = Math.abs(target - currentPosition);
        return error <= 0.2; // Returns true if within the tolerance range
    }

    public boolean isTiltMotorAtGoal() {
        return isTiltMotorAtGoal(m_desiredTarget);
    }
}
