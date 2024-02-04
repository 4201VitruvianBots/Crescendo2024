// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.FLYWHEEL;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final TalonFX[] m_flywheelMotors = {
    new TalonFX(CAN.flywheel1), new TalonFX(CAN.flywheel2)
  };

  private double m_percentOutput;
  private double flywheelPercentRatio = 1.0;
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  private final SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(FLYWHEEL.kS, FLYWHEEL.kV, FLYWHEEL.kA);
  private SimpleMotorFeedforward m_currentFeedForward = m_feedForward;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Flywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kV = FLYWHEEL.kV;
    config.Slot0.kP = FLYWHEEL.kP;
    config.Slot0.kI = FLYWHEEL.kI;
    config.Slot0.kD = FLYWHEEL.kD;
    CtreUtils.configureTalonFx(m_flywheelMotors[0], config);

    // flywheel motor 1
    m_flywheelMotors[1].setControl(new Follower(m_flywheelMotors[0].getDeviceID(), true));
  }

  // values that we set
  public void setPercentOutput(double percentOutput) {
    m_flywheelMotors[0].setControl(m_dutyCycleRequest.withOutput(percentOutput));
  }

  public void setRpmOutput(double rpm) {
    // Phoenix 6 uses rotations per second for velocity control
    var rps = rpm / 60.0;
    m_flywheelMotors[0].setControl(
        m_velocityRequest.withVelocity(rps).withFeedForward(m_currentFeedForward.calculate(rps)));
  }

  public void setPidValues(double v, double p, double i, double d) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Get the current motor configs to not erase everything
    m_flywheelMotors[0].getConfigurator().refresh(config);
    config.Slot0.kV = v;
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    CtreUtils.configureTalonFx(m_flywheelMotors[0], config);
  }

  public void setSimpleMotorFeedForward(double s, double v, double a) {
    m_currentFeedForward = new SimpleMotorFeedforward(s, v, a);
  }

  // values that we are pulling
  public double getPercentOutput() {
    return m_percentOutput;
  }

  private void updateLogger() {
    Logger.recordOutput("Flywheel/PercentOutput", getPercentOutput());
  }

  private void updateShuffleboard() {}

  @Override
  public void periodic() {
    updateShuffleboard();
    updateLogger();
  }
}
