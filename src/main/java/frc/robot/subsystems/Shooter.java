// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FLYWHEEL;

public class Shooter extends SubsystemBase {
  private final PWMTalonFX flywheelmotor1 = new PWMTalonFX(FLYWHEEL.flywheel1);
  private final TalonFX flywheelmotor2 = new TalonFX(FLYWHEEL.flywheel2);
  private final StaticBrake brakeControl = new StaticBrake();
  private final NeutralOut neutralControl = new NeutralOut();

  private final TalonFXConfiguration flywheelmotorconfig = new TalonFXConfiguration();

  private double flywheelmaxRPM = 1;
  private double gearRatio = 1.0 / 1.0;
  private double flywheelRPM;

  private double flywheelRPMratio = 1.0;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Shooter() {
    // flywheel motor 1
    flywheelmotor1.setInverted(true);
    // TO DO: change pid loop to effect rpm
    // flywheel motor 2
    flywheelmotor2.setInverted(true);
  }

  // values that we set
  public void maxRPM() {
    flywheelmotor1.set(flywheelmaxRPM);
    flywheelmotor2.set(flywheelmaxRPM * flywheelRPMratio);
    flywheelRPM = flywheelmaxRPM;
  }

  public void minRPM2(double flywheelRPM2) {
    flywheelmotor1.set(0);
    flywheelmotor2.set(0);
    flywheelRPM = 0;
  }

  // values that we are pulling
  public double getRPM1() {
    return flywheelRPM;
  }

  private void initShuffleboard() {}

  private void updateShuffleboard() {
    SmartDashboard.putNumber("RPM1", this.getRPM1());
  }

  @Override
  public void periodic() {
    this.updateShuffleboard();
  }
}
