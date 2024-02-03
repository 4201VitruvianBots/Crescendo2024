// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;

public class Climber extends SubsystemBase {
  private final TalonFX[] elevatorClimbMotors = {new TalonFX(CLIMBER.climbMotor1), new TalonFX(CLIMBER.climbMotor2)};
  private final StaticBrake brake = new StaticBrake();
  private final Follower follower = new Follower(0, false);

  private double m_desiredPositionMeters; // The height in meters our robot is trying to reach

  private double m_upperLimitMeters = CLIMBER.upperLimitMeters;
  private double m_lowerLimitMeters = CLIMBER.lowerLimitMeters;

  private CLIMBER_SETPOINT m_desiredSetpoint = CLIMBER_SETPOINT.FULL_RETRACT;
  private double m_elevatorDesiredSetpointMeters;
  private boolean elevatorClimbSate;
  private double holdPosition;

   // Testing value for mech2d
  public double m_mechHeight = 0.1;

  public DoubleSubscriber m_mechHeightSub;

  NetworkTable climberNtTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Climber");
  
  /** Creates a new climberMechanism. */
  public Climber() {
    // Initialize Test Values
    climberNtTab.getDoubleTopic("Climber Sim Test Height").publish().set(m_mechHeight);
    m_mechHeightSub =
      climberNtTab.getDoubleTopic("Climber Sim Test Height").subscribe(m_mechHeight);


    for (TalonFX motor : elevatorClimbMotors)
    motor.setControl(brake);
    
    elevatorClimbMotors[0].getConfigurator().apply(new TalonFXConfiguration());
    elevatorClimbMotors[0].setInverted(true);
    elevatorClimbMotors[1].setInverted(true);
    elevatorClimbMotors[1].setControl(follower);
  }

  public void setClimbState(boolean state) {
    elevatorClimbSate = state;
  }

  public boolean getClimbState() {
    return elevatorClimbSate;
  }

  //sets the percent ourput of the elevator based on its position
  public void setPercentOutput(double output, boolean enforceLimits) {
    if (enforceLimits){
      if (getHeightMeters() > getUpperLimitMeters())
        output = Math.min(output, 0);

      if (getHeightMeters() < getLowerLimitMeters())
        output = Math.max(output, 0);
    }

    elevatorClimbMotors[0].set(output);
  }

  public double getVelocityMetersPerSecond() {
    return elevatorClimbMotors[0].getRotorVelocity().getValueAsDouble() * CLIMBER.encoderCountsToMeters *10;
  }

  // gets the position of the climber in meters
  public double getHeightMeters() {
    return getHeightEncoderCounts() * CLIMBER.encoderCountsToMeters;
  }

  //gets the position of the climber in encoder counts
  public double getHeightEncoderCounts() {
    return elevatorClimbMotors[0].getRotorPosition().getValueAsDouble();
  }

  //sets position in meters
  public void setSensorPosition(double meters) {
    elevatorClimbMotors[0].setPosition(meters / CLIMBER.encoderCountsToMeters);
  }

  public void holdClimber() {
    elevatorClimbMotors[0].set(holdPosition);
  }

  public void setDesiredSetpoint(CLIMBER_SETPOINT desiredSetpoint) {
    m_desiredSetpoint = desiredSetpoint;
    setCLimberDesiredSetpoint(desiredSetpoint);
  }

  public void setCLimberDesiredSetpoint(CLIMBER_SETPOINT desiredSetpoint) {
    m_elevatorDesiredSetpointMeters = desiredSetpoint.getClimberSetpointMeters();
  }

  public void setDesiredPositionMeters(double meters) {
    m_desiredPositionMeters = meters;
  }

  public double getDesiredPostionMeters(double meters) {
    return m_desiredPositionMeters;
  }

  public void setHoldPosition(double position) {
    holdPosition = position;
  }

  public void setLowerLimitMeters(double meters) {
    m_lowerLimitMeters = meters;
  }

  public double getLowerLimitMeters() {
    return m_lowerLimitMeters;
  }

  public void setUpperLimitMeters(double meters) {
    m_upperLimitMeters = meters;
  }

  public double getUpperLimitMeters() {
    return m_upperLimitMeters;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
