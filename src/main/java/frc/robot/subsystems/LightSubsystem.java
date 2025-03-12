// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

public class LightSubsystem extends SubsystemBase {
    private final AddressableLEDBuffer m_ledBuffer;
    private final LEDPattern m_scrollingRainbow;
    private final ElevatorSubsystem m_elevator;
    private final Distance m_kLedSpacing;
    private LEDPattern m_evelatorPattern;
    private final LEDPattern m_rainbow;
    private final AddressableLED m_led;
    private float m_valor = 0;

    /** Creates a new LEDSubsystem. */
    public LightSubsystem(ElevatorSubsystem elevatorSubsystem) {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(9);

        // all hues at maximum saturation and half brightness

        m_rainbow = LEDPattern.rainbow(255, 128);

        // Our LED strip has a density of 120 LEDs per meter

        m_kLedSpacing = Meters.of(1 / 120.0);
        // Create a new pattern that scrolls the rainbow pattern across the LED strip,
        // moving at a speed
        // of 1 meter per second.
        m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), m_kLedSpacing);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        m_elevator = elevatorSubsystem;
    }

    public Command setConstantColor(int red, int green, int blue) {
        return runOnce(() -> {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, red, green, blue);
            }

            m_led.setData(m_ledBuffer);
        });
    }

    public Command setRainbow() {
        return runOnce(() -> {
            m_scrollingRainbow.applyTo(m_ledBuffer); // Update the buffer with the rainbow animation
            m_led.setData(m_ledBuffer); // Set the LEDs
        });
    }

    public void elevatorPattern() {
        LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue, Color.kRed);
        LEDPattern mask = LEDPattern.progressMaskLayer(() -> m_elevator.getPosition() / 1.3);
        m_evelatorPattern = base.mask(mask);

        m_evelatorPattern.applyTo(m_ledBuffer);

        m_led.setData(m_ledBuffer);
    }

    public Command increaseValor() {
        return Commands.run(() -> {
            m_valor = Math.min(m_valor + 1, 100);
        }, this);
    }

    public Command decreaseValor() {
        return Commands.run(() -> {
            m_valor = Math.max(m_valor - 1, 0);
        }, this);
    }

    public void setChoppedRainbow() {
        Map<Number, Color> maskSteps = Map.of(0, Color.kWhite, 0.5, Color.kBlack);
        LEDPattern base = LEDPattern.rainbow(255, 255);
        LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(100));

        LEDPattern pattern = base.mask(mask);

        // Apply the LED pattern to the data buffer
        pattern.applyTo(m_ledBuffer);

        // Write the data to the LED strip
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {
        if (!m_elevator.atTarget())
            elevatorPattern();
    }
}