// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CLIMBER;

public class Climber extends SubsystemBase {
  private final TalonFX[] elevatorClimbMotors = {new TalonFX(CLIMBER.climbMotor1), new TalonFX(CLIMBER.climbMotor2)};
  private final NeutralOut neutralControl = new NeutralOut();
  private final StaticBrake holdPosition = new StaticBrake();

  private double climberPosition = 0;
  private boolean elevatorClimbSate;


  
  /** Creates a new climberMechanism. */
  public Climber() {
    for (TalonFX motor : elevatorClimbMotors)
    motor.setControl(holdPosition);
    
    elevatorClimbMotors[0].getConfigurator().apply(new TalonFXConfiguration());
    elevatorClimbMotors[0].setInverted(true);
    elevatorClimbMotors[1].setInverted(true);
  }

  public void setElevatorClimbState(boolean state) {
    elevatorClimbSate = state;
  }

  public boolean getElevatorClimbState() {
    return elevatorClimbSate;
  }

  public void setElevatorMotorOutput(double percentOutput) {
    elevatorClimbMotors[0].set(percentOutput);
  }

  
  public double getElevatorClimbPosition() {
    return elevatorClimbMotors[0].getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
