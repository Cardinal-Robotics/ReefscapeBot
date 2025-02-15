// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Percent;

import java.util.Map;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;

public class LEDSubsystem extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    LEDPattern m_rainbow;
    Distance kLedSpacing;
    LEDPattern m_scrollingRainbow;
    LEDPattern m_evelatorPattern;
    LEDPattern m_ChoppedRainbow;

    /** Creates a new LEDSubsystem. */

    public LEDSubsystem() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(8);

        // all hues at maximum saturation and half brightness

        m_rainbow = LEDPattern.rainbow(255, 128);

        // Our LED strip has a density of 120 LEDs per meter

        kLedSpacing = Meters.of(1 / 120.0);
        // Create a new pattern that scrolls the rainbow pattern across the LED strip,
        // moving at a speed
        // of 1 meter per second.
        m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    private double temp = 0;

    public void setRed() {
        temp += 0.1;
        SmartDashboard.putNumber("setRed()", temp);
        /*
         * LEDPattern red = LEDPattern.solid(Color.kRed);
         * 
         * red.applyTo(m_ledBuffer);
         */

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 253, 11, 205);
        }

        // Set the data
        m_led.setData(m_ledBuffer);
    }

    public void setBlue() {
        temp += 1;
        SmartDashboard.putString("Change?", "yes");

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
        }

        m_led.setData(m_ledBuffer);
    }

    public void setRainbow() {
        // Update the buffer with the rainbow animation
        m_scrollingRainbow.applyTo(m_ledBuffer);
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }

    public void elevatorPattern(float valor) {
        LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue, Color.kRed);
        LEDPattern mask = LEDPattern.progressMaskLayer(() -> valor / 50);
        m_evelatorPattern = base.mask(mask);

        m_evelatorPattern.applyTo(m_ledBuffer);

        m_led.setData(m_ledBuffer);
    }

    public float increaseValor(float valor) {
        if (valor != 100.0) {
            valor++;
        }
        return valor;
    }

    public float decreaseValor(float valor) {
        if (valor != 0.0) {
            valor--;
        }
        return valor;
    }


           

    if (r < 255) {     

    
    

    i    r--;
        }
        return r;
    }

    public void set(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }

        m_led.setData(m_ledBuffer);
    }

    public void setChoppedRainbow() {
        Map<Number, Color> maskSteps = Map.of(0, Color.kWhite, 0.5, Color.kBlack);
        LEDPattern base = LEDPattern.rainbow(255, 255);
     

           

    LEDPattern pattern = base.mask(mask);


        // Wri
    e th

    

    rride

    // This method will be called once per sche
    

    
    