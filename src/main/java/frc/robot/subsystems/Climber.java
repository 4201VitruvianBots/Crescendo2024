// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final TalonFX[] elevatorClimbMotors = {
    new TalonFX(CAN.climbMotor1), new TalonFX(CAN.climbMotor2)
  };

  private final StatusSignal<Double> m_positionSignal =
      elevatorClimbMotors[0].getPosition().clone();
  private final StatusSignal<Double> m_positionSignal2 =
      elevatorClimbMotors[1].getPosition().clone();
  private final StatusSignal<Double> m_velocitySignal =
      elevatorClimbMotors[0].getVelocity().clone();
  private final StatusSignal<Double> m_velocitySignal2 =
      elevatorClimbMotors[1].getVelocity().clone();
  private final StatusSignal<Double> m_leftCurrentSignal =
      elevatorClimbMotors[0].getTorqueCurrent().clone();
  private final StatusSignal<Double> m_rightCurrentSignal =
      elevatorClimbMotors[1].getTorqueCurrent().clone();
  private final StatusSignal<Double> m_leftVoltageSignal =
      elevatorClimbMotors[0].getMotorVoltage().clone();
  private final StatusSignal<Double> m_rightVoltageSignal =
      elevatorClimbMotors[1].getMotorVoltage().clone();

  private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);

  private double m_desiredPositionMeters; // The height in meters our robot is trying to reach
  private final double m_upperLimitMeters = CLIMBER.upperLimitMeters;
  private final double m_lowerLimitMeters = CLIMBER.lowerLimitMeters;
  private CLIMBER_SETPOINT m_desiredSetpoint = CLIMBER_SETPOINT.FULL_RETRACT;

  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  // Controlled by open loop
  private double m_joystickInput;
  private boolean m_limitJoystickInput;
  private boolean m_enforceLimits;
  private boolean m_userSetpoint;

  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  private boolean elevatorClimbSate;

  private final ElevatorSim leftElevatorSim =
      new ElevatorSim(
          CLIMBER.gearbox,
          CLIMBER.gearRatio,
          CLIMBER.carriageMassKg,
          CLIMBER.sprocketRadiusMeters,
          CLIMBER.lowerLimitMeters,
          CLIMBER.upperLimitMeters,
          false,
          CLIMBER.lowerLimitMeters);
  private final ElevatorSim rightElevatorSim =
      new ElevatorSim(
          CLIMBER.gearbox,
          CLIMBER.gearRatio,
          CLIMBER.carriageMassKg,
          CLIMBER.sprocketRadiusMeters,
          CLIMBER.lowerLimitMeters,
          CLIMBER.upperLimitMeters,
          false,
          CLIMBER.lowerLimitMeters);

  private final TalonFXSimState m_simState1;
  private final TalonFXSimState m_simState2;

  /** Creates a new climberMechanism. */
  public Climber() {
    // Initialize Test Values
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = CLIMBER.gearRatio;
    config.Slot0.kP = CLIMBER.kP;
    config.Slot0.kI = CLIMBER.kI;
    config.Slot0.kD = CLIMBER.kD;
    config.Slot0.kA = CLIMBER.kA;
    config.Slot0.kV = CLIMBER.kV;

    config.MotionMagic.MotionMagicCruiseVelocity = 100;
    config.MotionMagic.MotionMagicAcceleration = 200;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CtreUtils.configureTalonFx(elevatorClimbMotors[0], config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CtreUtils.configureTalonFx(elevatorClimbMotors[1], config);

    m_simState1 = elevatorClimbMotors[0].getSimState();
    m_simState2 = elevatorClimbMotors[1].getSimState();
    m_simState1.Orientation = ChassisReference.CounterClockwise_Positive;
    m_simState2.Orientation = ChassisReference.Clockwise_Positive;
    //     elevatorClimbMotors[1].setControl(new Follower(elevatorClimbMotors[0].getDeviceID(),
    // true));

    SmartDashboard.putData(this);
  }

  public void setClimbState(boolean state) {
    elevatorClimbSate = state;
  }

  public boolean getClimbState() {
    return elevatorClimbSate;
  }

  public double getPercentOutputMotor1() {
    return elevatorClimbMotors[0].get();
  }

  public double getPercentOutputMotor2() {
    return elevatorClimbMotors[1].get();
  }

  private void setPercentOutput(double output) {
    setPercentOutput(output, false);
  }

  // sets the percent output of the elevator based on its position
  private void setPercentOutput(double output, boolean enforceLimits) {
    if (enforceLimits) {
      if (getHeightMetersMotor1() >= getUpperLimitMeters() - Units.inchesToMeters(1.2))
        output = Math.min(output, 0);

      if (getHeightMetersMotor1() <= getLowerLimitMeters() + Units.inchesToMeters(0.05))
        output = Math.max(output, 0);
    }

    elevatorClimbMotors[0].set(output);
    elevatorClimbMotors[1].set(output);
  }

  public double getAvgCurrentDraw() {
    return (m_leftCurrentSignal.getValue() + m_rightCurrentSignal.getValue()) * 0.5;
  }

  // gets the position of the climber in meters
  public double getHeightMetersMotor1() {
    return getMotor1Rotations() * CLIMBER.sprocketRotationsToMeters;
  }

  public double getHeightMetersMotor2() {
    return getMotor2Rotations() * CLIMBER.sprocketRotationsToMeters;
  }

  // gets the position of the climber in encoder counts
  public double getMotor1Rotations() {
    m_positionSignal.refresh();
    return m_positionSignal.getValue();
  }

  public double getMotor2Rotations() {
    m_positionSignal2.refresh();
    return m_positionSignal2.getValue();
  }

  public double getMotor1Voltage() {
    m_leftVoltageSignal.refresh();
    return m_leftVoltageSignal.getValue();
  }

  public double getMotor2Voltage() {
    m_rightVoltageSignal.refresh();
    return m_rightVoltageSignal.getValue();
  }

  // sets position in meters
  public void setSensorPosition(double meters) {
    elevatorClimbMotors[0].setPosition(meters / CLIMBER.sprocketRotationsToMeters);
  }

  public double getVelocityMetersPerSecond() {
    return m_velocitySignal.getValue() * CLIMBER.sprocketRotationsToMeters;
  }

  public void holdClimber() {
    setDesiredPositionMeters(getHeightMetersMotor1());
  }

  public double getDesiredSetpoint() {
    return m_desiredPositionMeters;
  }

  public void setDesiredPositionMeters(double setpoint) {
    m_desiredPositionMeters =
        MathUtil.clamp(setpoint, CLIMBER.lowerLimitMeters, CLIMBER.upperLimitMeters);
  }

  public double getDesiredPositionMeters() {
    return m_desiredPositionMeters;
  }

  // Sets the setpoint to our current height, effectively keeping the elevator in place.
  public void resetMotionMagicState() {
    m_desiredPositionMeters = getHeightMetersMotor1();
  }

  public double getLowerLimitMeters() {
    return m_lowerLimitMeters;
  }

  public double getUpperLimitMeters() {
    return m_upperLimitMeters;
  }

  public void setJoystickLimit(boolean limit) {
    m_limitJoystickInput = limit;
  }

  public void setJoystickY(double m_joystickY) {
    m_joystickInput = m_joystickY;
  }

  public boolean isUserControlled() {
    return m_joystickInput != 0 && !m_userSetpoint;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }

  // Sets the control state of the elevator
  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  // Returns the current control state enum
  public CONTROL_MODE getClosedLoopControlMode() {
    return m_controlMode;
  }

  public boolean isClosedLoopControl() {
    return getClosedLoopControlMode() == CONTROL_MODE.CLOSED_LOOP;
  }

  public void setClimberNeutralMode(NeutralModeValue mode) {
    if (mode == m_neutralMode) return;
    m_neutralMode = mode;
    elevatorClimbMotors[0].setNeutralMode(mode);
    elevatorClimbMotors[1].setNeutralMode(mode);
  }

  public NeutralModeValue getNeutralMode() {
    return m_neutralMode;
  }

  public void teleopInit() {
    resetMotionMagicState();
    setDesiredPositionMeters(getHeightMetersMotor1());
  }

  private void updateLogger() {
    Logger.recordOutput("Climber/Control Mode", getClosedLoopControlMode());
    Logger.recordOutput("Climber/Height MetersMotor1", getHeightMetersMotor1());
    Logger.recordOutput("Climber/Motor1 Rotations", getMotor1Rotations());
    Logger.recordOutput("Climber/Height MetersMotor2", getHeightMetersMotor2());
    Logger.recordOutput("Climber/Motor2 Rotations", getMotor2Rotations());
    Logger.recordOutput("Climber/Climb State", getClimbState());
    Logger.recordOutput("Climber/Motor1 Output", getPercentOutputMotor1());
    Logger.recordOutput("Climber/Motor2 Output", getPercentOutputMotor2());
    Logger.recordOutput("Climber/Setpoint", getDesiredSetpoint());
    Logger.recordOutput("Climber/Supply Current", getAvgCurrentDraw());

    Logger.recordOutput("Climber/Motor 1 Voltage", getMotor1Voltage());
    Logger.recordOutput("Climber/Motor 2 Voltage", getMotor2Voltage());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_controlMode) {
      case CLOSED_LOOP:
        elevatorClimbMotors[0].setControl(m_request.withPosition(m_desiredPositionMeters));
        //
        // elevatorClimbMotors[1].setControl(m_request.withPosition(m_desiredPositionMeters));
        break;
      case OPEN_LOOP:
      default:
        double percentOutput = m_joystickInput * CLIMBER.kPercentOutputMultiplier;

        // if (m_limitJoystickInput)
        // percentOutput = joystickYDeadband * CLIMBER.kLimitedPercentOutputMultiplier;
        //        if (m_enforceLimits) {
        //          if (getHeightMeters() >= getUpperLimitMeters() - Units.inchesToMeters(1.2))
        //            percentOutput = Math.min(percentOutput, 0);

        // TODO: Verify rotation to distance conversion before continuing
        setPercentOutput(percentOutput);
        break;
    }
    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    m_simState1.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_simState2.setSupplyVoltage(RobotController.getBatteryVoltage());

    leftElevatorSim.setInputVoltage(MathUtil.clamp(m_simState1.getMotorVoltage(), -12, 12));
    rightElevatorSim.setInputVoltage(MathUtil.clamp(m_simState2.getMotorVoltage(), -12, 12));

    leftElevatorSim.update(RobotTime.getTimeDelta());
    rightElevatorSim.update(RobotTime.getTimeDelta());

    m_simState1.setRawRotorPosition(
        leftElevatorSim.getPositionMeters()
            * CLIMBER.gearRatio
            / CLIMBER.sprocketRotationsToMeters);
    m_simState1.setRotorVelocity(
        leftElevatorSim.getVelocityMetersPerSecond()
            * CLIMBER.gearRatio
            / CLIMBER.sprocketRotationsToMeters);
    m_simState2.setRawRotorPosition(
        rightElevatorSim.getPositionMeters()
            * CLIMBER.gearRatio
            / CLIMBER.sprocketRotationsToMeters);
    m_simState2.setRotorVelocity(
        rightElevatorSim.getVelocityMetersPerSecond()
            * CLIMBER.gearRatio
            / CLIMBER.sprocketRotationsToMeters);
  }
}
