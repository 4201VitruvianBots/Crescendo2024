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

public class Shooter extends SubsystemBase {
  private final TalonFX flywheelmotor1 = new TalonFX(FLYWHEEL.flywheelmotor1);
  private final TalonFX iflywheelmotor2 = new TalonFX(FLYWHEEL.flywheelmotor2);
  private final StaticBrake brakeControl = new StaticBrake();
  private final NeutralOut neutralControl = new NeutralOut();
  
  private double flywheelRPM1;
  private double gearRatio = 1.0/1.0;
  private double flywhee2RPM2;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /** Creates a new Intkae. */
  public Shooter() {
    //TO DO: change pid loop to effect rpm
    // flywheel motor 1
    flywheelmotor1.setControl(neutralControl);
    flywheelmotor1.getConfigurator().apply(new TalonFXConfiguration());
    flywheelmotor1.setInverted(true);
    flywheelmotor1.config_kF(0, kF);
    flywheelmotor1.config_kP(0, kP);
    flywheelmotor1.config_kI(0, kI);
    flywheelmotor1.config_kD(0, kD);
    
    //TO DO: change pid loop to effect rpm
    // flywheel motor 2
    flywheelmotor2.setControl(neutralControl);
    flywheelmotor2.getConfigurator().apply(new TalonFXConfiguration());
    flywheelmotor2.setInverted(true);
    flywheelmotor2.config_kF(0, kF);
    flywheelmotor2.config_kP(0, kP);
    flywheelmotor2.config_kI(0, kI);
    flywheelmotor12.config_kD(0, kD);
  }

  //values that we set
  public void setMotorOutput1(double percentOutput) {
    flywheelmotor1.set(percentOutput);
  }

  public void setMotorOutput2(double percentOutput) {
    flywheelmotor2.set(percentOutput);
  }
  public void setRPM1(double flywheelRPM1){
    this.flywheelRPM1 = flywheelRPM1;
  }

  public void setRPM2(doule flywhee2RPM2) {
    this.flywheelRPM2 = flywheelRPM2;
  }

  //values that we are pulling
  public double getRPM1(){
    return flywheelRPM1;
  }

  public double getRPM2(){
    return flywheelRPM2;
  }

  
  private void initShuffleboard(){

  }

private void updateShuffleboard(){
    SmartDashboard.putNumber("RPM1", getRPM1);

}

  @Override
  public void periodic() {
    updateShuffleboard;
  }
}
