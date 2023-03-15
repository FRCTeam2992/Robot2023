// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.Color;
import frc.robot.Constants.LEDColors;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  private int ledsFrameCounter = 0;
  private int ledsLoopCounter = 0;

  private int[] colorChaseArr = {};

  public LEDs() {
    m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(17);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void showNextCycleColor(Color color1, Color color2, int frameCounter) {
    if (frameCounter / 3 == 0) {
      cjColorChaseFrame1(color1, color2);
    } else {
      cjColorChaseFrame3(color2, color1);
    }
  }

  public void setSingleLEDColor(int pixel, Color color) {
    m_ledBuffer.setRGB(pixel, color.r(), color.g(), color.b());
  }

  public void setLEDStripColor(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      setSingleLEDColor(i, color);
    }

    m_led.setData(m_ledBuffer);
  }

  public void geradColorSwap(Color color1, Color color2) {

    setSingleLEDColor(8, color1);
    for (int i = 1; i < 9; i++) {
      setSingleLEDColor(8 + i, color1);
      setSingleLEDColor(8 - i, color1);
    }

    setSingleLEDColor(8, color2);
    for (int i = 1; i < 9; i++) {
      setSingleLEDColor(8 + i, color2);
      setSingleLEDColor(8 - i, color2);
    }
  }

  public void geradColorChase2(Color color1, Color color2) {

    setSingleLEDColor(8, color2);
    for (int i = 1; i < 3; i++) {
      setSingleLEDColor(8 + i, color1);
      setSingleLEDColor(9 + i, color2);
      setSingleLEDColor(10 + i, color2);
      setSingleLEDColor(11 + i, color2);
      setSingleLEDColor(12 + i, color1);
      setSingleLEDColor(13 + i, color1);
      setSingleLEDColor(14 + i, color2);
      setSingleLEDColor(15 + i, color2);
      setSingleLEDColor(16 + i, color2);

      setSingleLEDColor(8 - i, color1);
      setSingleLEDColor(7 - i, color2);
      setSingleLEDColor(6 - i, color2);
      setSingleLEDColor(5 - i, color2);
      setSingleLEDColor(4 - i, color1);
      setSingleLEDColor(3 - i, color1);
      setSingleLEDColor(2 - i, color2);
      setSingleLEDColor(1 - i, color2);
      setSingleLEDColor(0 - i, color2);
    }

    for (int i = 1; i < 4; i++) {
      setSingleLEDColor(8 + i, color2);
      setSingleLEDColor(9 + i, color1);
      setSingleLEDColor(10 + i, color1);
      setSingleLEDColor(11 + i, color2);
      setSingleLEDColor(12 + i, color2);
      setSingleLEDColor(13 + i, color2);
      setSingleLEDColor(14 + i, color1);
      setSingleLEDColor(15 + i, color1);
      setSingleLEDColor(16 + i, color2);

      setSingleLEDColor(8 - i, color2);
      setSingleLEDColor(7 - i, color1);
      setSingleLEDColor(6 - i, color1);
      setSingleLEDColor(5 - i, color2);
      setSingleLEDColor(4 - i, color2);
      setSingleLEDColor(3 - i, color2);
      setSingleLEDColor(2 - i, color1);
      setSingleLEDColor(1 - i, color1);
      setSingleLEDColor(0 - i, color2);
    }

  }

  public void cjColorChaseFrame1(Color color1, Color color2) {
    Color[] frame = {
        color1, color1, color1,
        color2, color2, color2,
        color1, color1, color1, color1, color1,
        color2, color2, color2,
        color1, color1, color1
    };// 111 222 11111 222 111

    for (int i = 0; i < 17; i++) {
      setSingleLEDColor(i, frame[i]);
    }

    m_led.setData(m_ledBuffer);
  }

  public void cjColorChaseFrame2(Color color1, Color color2) {
    Color[] frame = {
        color1, color1,
        color2, color2, color2,
        color1, color1, color1,
        color2,
        color1, color1, color1,
        color2, color2, color2,
        color1, color1
    };// 11 222 111 2 111 222 11

    for (int i = 0; i < 17; i++) {
      setSingleLEDColor(i, frame[i]);
    }

    m_led.setData(m_ledBuffer);
  }

  public void cjColorChaseFrame3(Color color1, Color color2) {
    Color[] frame = {
        color1,
        color2, color2, color2,
        color1, color1, color1,
        color2, color2, color2,
        color1, color1, color1,
        color2, color2, color2,
        color1
    };// 1 222 111 222 111 222 1

    for (int i = 0; i < 17; i++) {
      setSingleLEDColor(i, frame[i]);
    }

    m_led.setData(m_ledBuffer);
  }

  public void mathColorChase(Color color1, Color color2) {
    setLEDStripColor(color1);

  }

  public void reidColorChaseFrame1(Color color1, Color color2) {

    // ------+++++------
    setSingleLEDColor(8, color1);

    setSingleLEDColor(7, color1);
    setSingleLEDColor(6, color1);

    setSingleLEDColor(9, color1);
    setSingleLEDColor(10, color1);

    // ---+++-----+++---
    setSingleLEDColor(5, color2);
    setSingleLEDColor(4, color2);
    setSingleLEDColor(3, color2);

    setSingleLEDColor(11, color2);
    setSingleLEDColor(12, color2);
    setSingleLEDColor(13, color2);

    // +++-----------+++
    setSingleLEDColor(2, color1);
    setSingleLEDColor(1, color1);
    setSingleLEDColor(0, color1);

    setSingleLEDColor(14, color1);
    setSingleLEDColor(15, color1);
    setSingleLEDColor(16, color1);

    m_led.setData(m_ledBuffer);
  }

  public void reidColorChaseFrame2(Color color1, Color color2) {
    // --------+--------
    setSingleLEDColor(8, color2);

    // -----+++-+++-----
    setSingleLEDColor(7, color1);
    setSingleLEDColor(6, color1);
    setSingleLEDColor(5, color1);

    setSingleLEDColor(9, color1);
    setSingleLEDColor(10, color1);
    setSingleLEDColor(11, color1);

    // --+++-------+++--
    setSingleLEDColor(4, color2);
    setSingleLEDColor(3, color2);
    setSingleLEDColor(2, color2);

    setSingleLEDColor(12, color2);
    setSingleLEDColor(13, color2);
    setSingleLEDColor(14, color2);

    // ++-------------++
    setSingleLEDColor(1, color1);
    setSingleLEDColor(0, color1);

    setSingleLEDColor(15, color1);
    setSingleLEDColor(16, color1);

    m_led.setData(m_ledBuffer);
  }

  public void reidColorChaseFrame3(Color color1, Color color2) {
    // -------+++-------
    setSingleLEDColor(8, color2);
    setSingleLEDColor(7, color2);
    setSingleLEDColor(9, color2);

    // ----+++---+++----
    setSingleLEDColor(6, color1);
    setSingleLEDColor(5, color1);
    setSingleLEDColor(4, color1);

    setSingleLEDColor(10, color1);
    setSingleLEDColor(11, color1);
    setSingleLEDColor(12, color1);

    // -+++---------+++-
    setSingleLEDColor(3, color2);
    setSingleLEDColor(2, color2);
    setSingleLEDColor(1, color2);

    setSingleLEDColor(13, color2);
    setSingleLEDColor(14, color2);
    setSingleLEDColor(15, color2);

    // +---------------+
    setSingleLEDColor(0, color1);

    setSingleLEDColor(16, color1);

    m_led.setData(m_ledBuffer);
  }
}
