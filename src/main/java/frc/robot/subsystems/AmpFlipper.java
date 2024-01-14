// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AMP;
import frc.robot.constants.CAN;
import frc.robot.utils.CtreUtils;

public class AmpFlipper extends SubsystemBase {
  /** Creates a new AmpFlipper. */
  private final TalonFX flipperMotor = new TalonFX(CAN.ampFlipper);

  private final PositionVoltage m_position = new PositionVoltage(0);

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(AMP.kMaxFlipperVelocity, AMP.kMaxFlipperAcceleration);
  private final TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private double m_desiredAngleRadians = 0;

  public AmpFlipper() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kS = AMP.kS;
    config.Slot0.kV = AMP.kV;
    config.Slot0.kP = AMP.kP;
    config.Slot0.kI = AMP.kI;
    config.Slot0.kD = AMP.kD;
    CtreUtils.configureTalonFx(flipperMotor, config);
  }

  public void setPercentOutput(double speed) {
    flipperMotor.set(speed);
  }

  public void setDesiredSetpointRadians(double radians) {
    m_desiredAngleRadians = radians;
    m_goal = new TrapezoidProfile.State(radians * AMP.kRadiansToRotations, 0);
  }

  public double getDesiredSetpointRadians() {
    return m_desiredAngleRadians;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // periodic, update the profile setpoint for 20 ms loop time
    m_setpoint = m_profile.calculate(RobotTime.getTimeDelta(), m_setpoint, m_goal);
    // apply the setpoint to the control request
    m_position.Position = m_setpoint.position;
    m_position.Velocity = m_setpoint.velocity;
    flipperMotor.setControl(m_position);
  }
}
