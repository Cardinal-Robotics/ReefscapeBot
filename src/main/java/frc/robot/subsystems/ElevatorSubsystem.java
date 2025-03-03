// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorTarget;
import frc.robot.RobotContainer.InteractionState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;
import frc.robot.Robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_master = new SparkMax(ElevatorConstants.kMasterMotorId, MotorType.kBrushless);
    // DO NOT CHANGE TO FOLLOWER
    private final SparkMax m_slave = new SparkMax(ElevatorConstants.kSlaveMotorId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_master.getEncoder();

    // Feedforward stuff (if needed)

    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
    private final ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(
            ElevatorConstants.kElevatorKs,
            ElevatorConstants.kElevatorKg,
            ElevatorConstants.kElevatorKv);

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    private ElevatorSim m_elevatorSim;
    private SparkMaxSim m_motorSim;

    @Override
    public void periodic() { // 49 + 3/8
        SmartDashboard.putNumber("ElevatorSubsystem::getPosition", getPosition());

        m_master.getClosedLoopController().setReference(SmartDashboard.getNumber("ElevatorHeight", getPosition()),
                ControlType.kPosition);

        m_setpoint = m_profile.calculate(0.02,
                new TrapezoidProfile.State(getPosition(), m_master.getEncoder().getVelocity()), m_goal);
        /*
         * m_master.getClosedLoopController().setReference(
         * m_setpoint.position,
         * ControlType.kPosition,
         * ClosedLoopSlot.kSlot0,
         * m_elevatorFeedforward.calculate(m_setpoint.velocity));
         */
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_motorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);
        m_elevatorSim.update(0.020);

        m_master.getClosedLoopController().setReference(SmartDashboard.getNumber("ElevatorHeight", 0),
                ControlType.kPosition);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    /**
     * Index 0 returns the position of the inner stage while index 1 returns the
     * position of the middle stage
     */
    public Pose3d[] getSimulatedElevatorPositions() {
        double middleStageHight = m_elevatorSim.getPositionMeters();
        double innerStageHight = middleStageHight
                + (Units.inchesToMeters(32.5) * (middleStageHight / Units.inchesToMeters(37.5)));

        return new Pose3d[] {
                new Pose3d(0, 0, middleStageHight, Rotation3d.kZero),
                new Pose3d(0, 0, innerStageHight, Rotation3d.kZero)
        };
    }

    public ElevatorSubsystem() {
        SmartDashboard.putNumber("ElevatorHeight", 0);
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        masterConfig.idleMode(IdleMode.kCoast);

        masterConfig.encoder.positionConversionFactor(ElevatorConstants.kPositionConversionFactor);
        masterConfig.encoder.velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

        masterConfig.closedLoop
                .pid(ElevatorConstants.kElevatorP,
                        ElevatorConstants.kElevatorI,
                        ElevatorConstants.kElevatorD)
                .outputRange(-.7, .7);

        SparkMaxConfig slaveConfig = new SparkMaxConfig();
        slaveConfig
                .follow(ElevatorConstants.kMasterMotorId, true)
                .idleMode(IdleMode.kCoast);

        slaveConfig.encoder.positionConversionFactor(ElevatorConstants.kPositionConversionFactor);

        m_master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // m_slave.configure(slaveConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kNoPersistParameters); // Look up
        // later

        // Simulation code
        if (!Robot.isSimulation())
            return;

        m_motorSim = new SparkMaxSim(m_master, DCMotor.getNEO(2));

        m_elevatorSim = new ElevatorSim(DCMotor.getNEO(2),
                20,
                18.0,
                Units.inchesToMeters(1.432) / 2,
                0,
                Units.inchesToMeters(37.5),
                false,
                0,
                0.001,
                0.0);
    }

    // Getters & setters
    public double getPosition() {
        return m_encoder.getPosition();
    }

    // --- Handles elevator target
    public void setElevatorGoal(ElevatorTarget goal) {
        double target = ElevatorPositions.kElevatorPositionAlgaeScore; // idle position
        switch (goal) {
            case CoralIntake:
                target = ElevatorPositions.kElevatorPositionCoralIntake;
                break;
            case AlgaeScore:
                target = ElevatorPositions.kElevatorPositionAlgaeScore;
                break;
            case L1:
                target = ElevatorPositions.kElevatorPositionCoralL1;
                break;
            case L2:
                target = (RobotContainer.interactionState == InteractionState.Algae)
                        ? ElevatorPositions.kElevatorPositionAlgaeL2
                        : ElevatorPositions.kElevatorPositionCoralL2;
                break;
            case L3:
                target = (RobotContainer.interactionState == InteractionState.Algae)
                        ? ElevatorPositions.kElevatorPositionAlgaeL3
                        : ElevatorPositions.kElevatorPositionCoralL3;
                break;
            case L4:
                target = ElevatorPositions.kElevatorPositionCoralL4;
                break;
        }

        // m_goal = new TrapezoidProfile.State(target, 0);
        SmartDashboard.putNumber("ElevatorHeight", target);
        m_master.getClosedLoopController().setReference(target, ControlType.kPosition);
    }
}
