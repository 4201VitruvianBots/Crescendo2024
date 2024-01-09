// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;

public class Flywheel extends SubsystemBase {
  private final TalonFX flywheelmotor1 = new TalonFX(FLYWHEEL.flywheelmotor1);
  private final TalonFX iflywheelmotor2 = new TalonFX(FLYWHEEL.flywheelmotor2);
  private final StaticBrake brakeControl = new StaticBrake();
  private final NeutralOut neutralControl = new NeutralOut();

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /** Creates a new Intkae. */
  public Intake() {
    // flywheel motor 1
    flywheelmotor1.setControl(neutralControl);
    flywheelmotor1.getConfigurator().apply(new TalonFXConfiguration());
    flywheelmotor1.setInverted(false);

    // flywheel motor 2
    flywheelmotor2.setControl(neutralControl);
    flywheelmotor2.getConfigurator().apply(new TalonFXConfiguration());
    flywheelmotor2.setInverted(true);
  }

  public void setMotorOutput1(double percentOutput) {
    flywheelmotor1.set(percentOutput);
  }

  public void setMotorOutput2(double percentOutput) {
    flywheelmotor2.set(percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
