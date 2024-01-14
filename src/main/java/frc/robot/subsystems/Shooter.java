// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.FLYWHEEL.gearRatio;
import static frc.robot.constants.FLYWHEEL.maxRPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(CAN.flywheel1);
  private final TalonFX motor2 = new TalonFX(CAN.flywheel2);

  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  private double flywheelRPM;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Shooter() {
    // flywheel motor 1
    var motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.SensorToMechanismRatio = gearRatio;
    CtreUtils.configureTalonFx(motor1, motorConfig);
    motor1.setInverted(true);

    // TO DO: change pid loop to effect rpm
    // flywheel motor 2
    CtreUtils.configureTalonFx(motor2, motorConfig);
    motor2.setInverted(false);
  }

  // values that we set
  public void setMaxRPM() {
    motor1.setControl(velocityControl.withVelocity(maxRPM));
    motor2.setControl(velocityControl.withVelocity(maxRPM));
  }

  public void setMinRPM() {
    motor1.setControl(velocityControl);
    motor2.setControl(velocityControl);
  }

  // values that we are pulling
  public double getRPM() {
    return motor1.getVelocity().getValue();
  }

  private void updateShuffleboard() {
    // SmartDashboard.putNumber("RPM1", this.getRPM());
  }

  private void updateLog() {
    Logger.recordOutput("Flywheel/RPM1", getRPM());
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    updateLog();
  }
}
