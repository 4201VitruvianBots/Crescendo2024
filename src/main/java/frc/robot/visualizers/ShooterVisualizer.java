// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.SHOOTER;

/** Add your docs here. */
public class ShooterVisualizer {
  private MechanismLigament2d m_spinner;
  private MechanismLigament2d m_side1;
  private MechanismLigament2d m_side2;
  private MechanismLigament2d m_side3;
  private MechanismLigament2d m_side4;
  private MechanismLigament2d m_side5;
  private MechanismLigament2d m_side6;
  private MechanismLigament2d m_side7;
  private MechanismLigament2d m_side8;

  private void initSides() {
    m_side1 =
        m_spinner.append(
            new MechanismLigament2d(
                "side1", SHOOTER.flywheelSize, 112.5, 3, new Color8Bit(Color.kDimGray)));
    m_side2 =
        m_side1.append(
            new MechanismLigament2d(
                "side2", SHOOTER.flywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
    m_side3 =
        m_side2.append(
            new MechanismLigament2d(
                "side3", SHOOTER.flywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
    m_side4 =
        m_side3.append(
            new MechanismLigament2d(
                "side4", SHOOTER.flywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
    m_side5 =
        m_side4.append(
            new MechanismLigament2d(
                "side5", SHOOTER.flywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
    m_side6 =
        m_side5.append(
            new MechanismLigament2d(
                "side6", SHOOTER.flywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
    m_side7 =
        m_side6.append(
            new MechanismLigament2d(
                "side7", SHOOTER.flywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
    m_side8 =
        m_side7.append(
            new MechanismLigament2d(
                "side8", SHOOTER.flywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  }

  public ShooterVisualizer(String name, MechanismRoot2d root2d) {
    m_spinner =
        root2d.append(
            new MechanismLigament2d(
                name, Units.inchesToMeters(1.7), 0, 0, new Color8Bit(Color.kAliceBlue)));

    initSides();
  }

  public ShooterVisualizer(String name, MechanismLigament2d ligament2d) {
    m_spinner =
        ligament2d.append(
            new MechanismLigament2d(
                name, Units.inchesToMeters(1.7), 0, 0, new Color8Bit(Color.kAliceBlue)));

    initSides();
  }

  public MechanismLigament2d getSpinnerLigament() {
    return m_spinner;
  }

  public MechanismLigament2d[] getSideLigaments() {
    return new MechanismLigament2d[] {
      m_side1, m_side2, m_side3, m_side4, m_side5, m_side6, m_side7, m_side8
    };
  }

  public void update(double rpm) {
    if (m_spinner != null) {
      m_spinner.setAngle(m_spinner.getAngle() - 360 * rpm / 60 * 0.2);
    } else {
      System.out.println("m_bottomFlywheel is null");
    }
  }
}
