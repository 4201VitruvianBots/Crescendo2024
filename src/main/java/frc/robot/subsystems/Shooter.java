// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;

public class Shooter extends SubsystemBase {
  private final TalonFX flywheelmotor1 = new TalonFX(CAN.flywheel1);
  private final TalonFX flywheelmotor2 = new TalonFX(CAN.flywheel2);
  private double m_rpm;
  private double m_headingOffset;
  private double flywheelRPMRatio = 1.0;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Shooter() {
    // flywheel motor 1
    flywheelmotor1.setInverted(true);
    // flywheel motor 2
    flywheelmotor2.setInverted(false);
  }

  // values that we set
  public void setRPM(double m_rpm) {
    flywheelmotor1.set(m_rpm);
    flywheelmotor2.set(m_rpm * flywheelRPMRatio);
  }

  // values that we are pulling
  public double getRPM1() {
    return m_rpm;
  }

  public double getRPM2() {
    return m_rpm * flywheelRPMRatio;
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("RPM1", this.getRPM1());
    SmartDashboard.putNumber("RPM2", this.getRPM2());
  }

  @Override
  public void periodic() {
    this.updateShuffleboard();
  }
}
