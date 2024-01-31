// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AMP;
import frc.robot.constants.CAN;
import frc.robot.constants.ROBOT;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX m_armMotor = new TalonFX(CAN.armMotor);

  private TalonFXSimState m_simState = m_armMotor.getSimState();

  private final PositionVoltage m_position = new PositionVoltage(0);

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(AMP.kMaxArmVelocity, AMP.kMaxArmAcceleration);
  private final TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private double m_desiredRotations = 0;

  private Timer m_simTimer = new Timer();
  private double lastSimTime;

  // Simulation setup
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          AMP.gearBox,
          AMP.gearRatio,
          SingleJointedArmSim.estimateMOI(AMP.length, AMP.mass),
          AMP.length,
          AMP.minAngleRadians,
          AMP.maxAngleRadians,
          false,
          AMP.minAngleRadians);

  private ROBOT.CONTROL_MODE m_controlMode = ROBOT.CONTROL_MODE.CLOSED_LOOP;

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kS = AMP.kS;
    config.Slot0.kV = AMP.kV;
    config.Slot0.kP = AMP.kP;
    config.Slot0.kI = AMP.kI;
    config.Slot0.kD = AMP.kD;
    config.Feedback.SensorToMechanismRatio = AMP.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CtreUtils.configureTalonFx(m_armMotor, config);

    // Simulation setup
    lastSimTime = m_simTimer.get();
    m_simTimer.start();
    SmartDashboard.putData(this);
    
    m_armMotor.setPosition(Units.radiansToRotations(AMP.minAngleRadians));
    setDesiredSetpointRotations(Units.radiansToRotations(AMP.minAngleRadians));
  }

  public void setPercentOutput(double speed) {
    m_armMotor.set(speed);
  }

  public double getPercentOutput() {
    return m_armMotor.get();
  }

  public void setDesiredSetpointRotations(double rotations) {
    m_desiredRotations = rotations;
    m_goal = new TrapezoidProfile.State(rotations, 0);
  }

  public double getDesiredSetpointRotations() {
    return m_desiredRotations;
  }

  public double getAngleDegrees() {
    return Units.rotationsToDegrees(m_armMotor.getPosition().getValue());
  }

  public void setControlMode(ROBOT.CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  public TalonFX getMotor() {
    return m_armMotor;
  }

  private void updateLogger() {
    Logger.recordOutput("Arm/DesiredAngle", Units.rotationsToDegrees(m_desiredRotations));
    Logger.recordOutput("Arm/CurrentAngle", getAngleDegrees());
    Logger.recordOutput("Arm/DesiredSetpoint", Units.rotationsToDegrees(m_setpoint.position));
    Logger.recordOutput("Arm/PercentOutput", m_armMotor.get());
  }

  @Override
  public void periodic() {
    switch (m_controlMode) {
      case CLOSED_LOOP:
        // This method will be called once per scheduler run
        // periodic, update the profile setpoint for 20 ms loop time
        m_setpoint = m_profile.calculate(RobotTime.getTimeDelta(), m_setpoint, m_goal);
        // apply the setpoint to the control request
        m_position.Position = m_setpoint.position;
        m_position.Velocity = m_setpoint.velocity;
        m_armMotor.setControl(m_position);
        break;
      default:
      case OPEN_LOOP:
        break;
    }

    updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    // Set supply voltage of flipper motor
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_armSim.setInputVoltage(MathUtil.clamp(m_simState.getMotorVoltage(), -12, 12));
    
    double dt = m_simTimer.get() - lastSimTime;
    m_armSim.update(dt);
    lastSimTime = m_simTimer.get();

    m_simState.setRawRotorPosition(
        Units.radiansToRotations(m_armSim.getAngleRads()) * AMP.gearRatio);

    m_simState.setRotorVelocity(
        Units.radiansToRotations(m_armSim.getVelocityRadPerSec()) * AMP.gearRatio);
  }
}
