// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

// For now not used due to incompatibility with Mechanism2d
/** An extension of MechanismLigament2d that visualizes the speed of a particular motor. */
public class MotorLigament2d extends MechanismLigament2d {
  MotorController m_motor;
  Color8Bit m_originalColor;
  
  /**
   * Create a new ligament.
   *
   * @param name      The ligament name.
   * @param length    The ligament length.
   * @param angle     The ligament angle in degrees.
   * @param lineWidth The ligament's line width.
   * @param color     The ligament's color.
   * @param motor     The motor that controls this ligament.
   */
  public MotorLigament2d(
      String name, double length, double angle, double lineWidth, Color8Bit color, MotorController motor) {
    super(name, length, angle, lineWidth, color);
    m_motor = motor;
    m_originalColor = color;
  }

  /**
   * Create a new ligament with the default color (orange) and thickness (6).
   *
   * @param name   The ligament's name.
   * @param length The ligament's length.
   * @param angle  The ligament's angle relative to its parent in degrees.
   * @param motor  The motor that controls this ligament.
   */
  public MotorLigament2d(String name, double length, double angle, MotorController motor) {
    this(name, length, angle, 10, new Color8Bit(235, 137, 52), motor);
  }
  
  /**
   * Set the ligament color.
   *
   * @param color the color of the line
   */
  @Override
  public synchronized void setColor(Color8Bit color) {
    m_originalColor = color;
    super.setColor(color);
  }
  
  public void updateColor() {
    double deltaBrightness = Math.abs(m_motor.get()) * 75;
    
    Color8Bit color = new Color8Bit(
        m_originalColor.red + (int) deltaBrightness,
        m_originalColor.green + (int) deltaBrightness,
        m_originalColor.blue + (int) deltaBrightness
    );
    
    super.setColor(color);
  }
}
