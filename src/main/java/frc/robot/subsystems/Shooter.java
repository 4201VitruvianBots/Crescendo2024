// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.FLYWHEEL;
import frc.robot.utils.CtreUtils;

public class Shooter extends SubsystemBase {

  private final TalonFX[] flywheelmotors = {new TalonFX(CAN.flywheel1), new TalonFX(CAN.flywheel2)};

  private double m_percentOutput;
  private double flywheelPercentRatio = 1.0;
  final DutyCycleOut m_request = new DutyCycleOut(0);

  private final SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(FLYWHEEL.kG, FLYWHEEL.kV, FLYWHEEL.kA);
  private SimpleMotorFeedforward m_currentFeedForward = m_feedForward;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Shooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kV = FLYWHEEL.kV;
    config.Slot0.kP = FLYWHEEL.kP;
    config.Slot0.kI = FLYWHEEL.kI;
    config.Slot0.kD = FLYWHEEL.kD;
    CtreUtils.configureTalonFx(flywheelmotors[0], config);

    // flywheel motor 1
    flywheelmotors[1].setControl(new Follower(flywheelmotors[0].getDeviceID(), true));
  }

  // values that we set
  public void setPercentOutput(double m_percentOutput) {

    flywheelmotors[0].setControl(m_request.withOutput(m_percentOutput));
  }

  public void setPIDvalues(double v, double p, double i, double d) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kV = v;
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    CtreUtils.configureTalonFx(flywheelmotors[0], config);
  }

  public void setSimpleMotorFeedForward(double g, double v, double a) {
    m_currentFeedForward = new SimpleMotorFeedforward(g, v, a);
  }

  // values that we are pulling
  public double getPercentOutput() {
    return m_percentOutput;
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("getPercentOutput", this.getPercentOutput());
  }

  @Override
  public void periodic() {
    this.updateShuffleboard();
  }
}
