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

public class Intake extends SubsystemBase {
  private final TalonFX intakemotor1 = new TalonFX(CAN.intakemotor1);
  private final TalonFX intakemotor2 = new TalonFX(CAN.intakemotor2);
  private final StaticBrake brakeControl = new StaticBrake();
  private final NeutralOut neutralControl = new NeutralOut();

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /** Creates a new Intkae. */
  public Intake() {
    // intake motor 1
    intakemotor1.setControl(brakeControl);
    intakemotor1.getConfigurator().apply(new TalonFXConfiguration());
    intakemotor1.setInverted(true);

    // intake motor 2
    intakemotor2.setControl(brakeControl);
    intakemotor2.getConfigurator().apply(new TalonFXConfiguration());
    intakemotor2.setInverted(true);
  }

  public void setMotorOutput1(double percentOutput) {
    intakemotor1.set(percentOutput);
  }

  public void setMotorOutput2(double percentOutput) {
    intakemotor1.set(percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
