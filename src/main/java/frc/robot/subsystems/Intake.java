// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;
// import frc.robot.constants.INTAKE.STATE;
import frc.robot.constants.ROBOT;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private boolean m_isIntaking = false;

  // Disabled until sensor installed
  DigitalInput distanceSensorDigitalInput = new DigitalInput(0);
  DigitalInput distanceSensorDigitalInput2 = new DigitalInput(1);

  // private STATE m_state = STATE.NONE;

  private final TalonFX intakeMotor1 = new TalonFX(CAN.intakeMotor1);
  private final TalonFXSimState m_intakeMotor1SimState = intakeMotor1.getSimState();
  private final TalonFX intakeMotor2 = new TalonFX(CAN.intakeMotor2);
  private final TalonFXSimState m_intakeMotor2SimState = intakeMotor2.getSimState();
  private final DCMotorSim m_intakeMotor1Sim =
      new DCMotorSim(INTAKE.intake1Gearbox, INTAKE.gearRatio, INTAKE.Inertia);
  private final DCMotorSim m_intakeMotor2Sim =
      new DCMotorSim(INTAKE.intake2Gearbox, INTAKE.gearRatio, INTAKE.Inertia);

  public Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = INTAKE.kP;
    config.Slot0.kI = INTAKE.kI;
    config.Slot0.kD = INTAKE.kD;
    config.Feedback.SensorToMechanismRatio = INTAKE.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CtreUtils.configureTalonFx(intakeMotor1, config);
    TalonFXConfiguration configBack = new TalonFXConfiguration();
    configBack.Slot0.kP = INTAKE.kP;
    configBack.Slot0.kI = INTAKE.kI;
    configBack.Slot0.kD = INTAKE.kD;
    configBack.Feedback.SensorToMechanismRatio = INTAKE.gearRatio;
    configBack.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configBack.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CtreUtils.configureTalonFx(intakeMotor2, configBack);

    SmartDashboard.putData(this);
  }

  public void setSpeed(double speed1, double speed2) {
    intakeMotor1.set(speed1);
    intakeMotor2.set(speed2);
  }

  public double getSpeed() {
    return intakeMotor1.get();
  }

  public double getRpm() {
    return intakeMotor1.getVelocity().getValue() * 60.0;
  }

  public void setIntakeState(boolean state) {
    m_isIntaking = state;
  }

  public boolean getIntakeState() {
    return m_isIntaking;
  }

  public boolean getSensorInput1() {
    // Disabled until sensor installed
    return !distanceSensorDigitalInput.get();
  }

  public boolean getSensorInput2() {
    // Disabled until sensor installed
    return !distanceSensorDigitalInput2.get();
  }

  public boolean checkEitherIntakeSensorActive() {
    return getSensorInput1() || getSensorInput2();
  }

  public boolean checkBothIntakeSensorActive() {
    return getSensorInput1() && getSensorInput2();
  }

  public boolean checkBothIntakeSensorClear() {
    return !getSensorInput1() && !getSensorInput2();
  }

  public void updateSmartDashboard() {}

  public void updateLogger() {
    Logger.recordOutput("Intake/Motor1 Velocity", intakeMotor1.getVelocity().getValue());
    Logger.recordOutput("Intake/Motor2 Velocity", intakeMotor2.getVelocity().getValue());
    Logger.recordOutput("Intake/Motor1 Output", intakeMotor2.getMotorVoltage().getValue() / 12.0);
    Logger.recordOutput("Intake/Motor2 Output", intakeMotor2.getMotorVoltage().getValue() / 12.0);
    Logger.recordOutput("Intake/Motor1 Current", intakeMotor2.getTorqueCurrent().getValue());
    Logger.recordOutput("Intake/Motor2 Current", intakeMotor2.getTorqueCurrent().getValue());
    Logger.recordOutput("Intake/BeamBreak1 state", getSensorInput1());
    Logger.recordOutput("Intake/BeamBreak2 state", getSensorInput2());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    m_intakeMotor1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_intakeMotor2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_intakeMotor2Sim.setInputVoltage(
        MathUtil.clamp(m_intakeMotor2SimState.getMotorVoltage(), -12, 12));
    m_intakeMotor1Sim.setInputVoltage(
        MathUtil.clamp(m_intakeMotor1SimState.getMotorVoltage(), -12, 12));

    m_intakeMotor2Sim.update(RobotTime.getTimeDelta());
    m_intakeMotor1Sim.update(RobotTime.getTimeDelta());

    m_intakeMotor2SimState.setRawRotorPosition(
        m_intakeMotor2Sim.getAngularPositionRotations() * INTAKE.gearRatio);
    m_intakeMotor2SimState.setRotorVelocity(
        m_intakeMotor2Sim.getAngularVelocityRPM() * INTAKE.gearRatio / 60.0);
    m_intakeMotor1SimState.setRawRotorPosition(
        m_intakeMotor1Sim.getAngularPositionRotations() * INTAKE.gearRatio);
    m_intakeMotor1SimState.setRotorVelocity(
        m_intakeMotor1Sim.getAngularVelocityRPM() * INTAKE.gearRatio / 60.0);
  }
}
