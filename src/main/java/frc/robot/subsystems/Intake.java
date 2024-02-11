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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;
import frc.robot.constants.SWERVE;
import frc.robot.constants.INTAKE.INTAKE_STATE;
import frc.robot.constants.ROBOT;
import frc.robot.utils.CtreUtils;

import static frc.robot.simulation.SimConstants.kMotorResistance;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private boolean m_isIntaking = false;

  private INTAKE_STATE m_state = INTAKE_STATE.NONE;

  private final TalonFX intakeMotor1 = new TalonFX(CAN.intakeMotor1);
  private final TalonFXSimState m_intakeMotor1SimState = intakeMotor1.getSimState();
  private final TalonFX intakeMotor2 = new TalonFX(CAN.intakeMotor2);
  private final TalonFXSimState m_intakeMotor2SimState = intakeMotor2.getSimState();
  
  private DCMotorSim m_intakeMotor1Sim=  
      new DCMotorSim(
         INTAKE.intake1Gearbox,
         INTAKE.gearRatio,
         INTAKE.Inertia);
  private DCMotorSim m_intakeMotor2Sim=  
      new DCMotorSim(
         INTAKE.intake2Gearbox,
         INTAKE.gearRatio,
         INTAKE.Inertia);
  public Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = INTAKE.kP;
    config.Slot0.kI = INTAKE.kI;
    config.Slot0.kD = INTAKE.kD;
    config.Feedback.SensorToMechanismRatio = INTAKE.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CtreUtils.configureTalonFx(intakeMotor1, config);
    CtreUtils.configureTalonFx(intakeMotor2, config);
  }

  public void setSpeed(double speed1, double speed2) {
    intakeMotor1.set(speed1);
    intakeMotor2.set(speed2);
  }

  public double getSpeed() {
    return intakeMotor1.get();
  }

  public void setIntaking(boolean isIntaking) {
    m_isIntaking = isIntaking;
  }

  public boolean isIntaking() {
    return m_isIntaking;
  }

  //   public void setIntakingState(INTAKE_STATE speed) {
  //     m_state = speed;
  //   }

  public INTAKE_STATE getIntakeState() {
    return m_state;
  }
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_intakeMotor1Sim.getCurrentDrawAmps(), m_intakeMotor1Sim.getCurrentDrawAmps()));

    m_intakeMotor1SimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_intakeMotor1Sim.getCurrentDrawAmps() * kMotorResistance);
    m_intakeMotor2SimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_intakeMotor2Sim.getCurrentDrawAmps() * kMotorResistance);

    m_intakeMotor2Sim.setInputVoltage(
        MathUtil.clamp(m_intakeMotor2SimState.getMotorVoltage(), -12, 12));
    m_intakeMotor1Sim.setInputVoltage(MathUtil.clamp(m_intakeMotor1SimState.getMotorVoltage(), -12, 12));

    //    System.out.printf("DT: %.2f\tDrive Rotations: %.2f\tDrive RPM: %.2f\n", dt,
    // m_intakeMotor2Sim.getAngularPositionRotations(), m_intakeMotor2Sim.getAngularVelocityRPM());

    m_intakeMotor2SimState.setRawRotorPosition(
        m_intakeMotor2Sim.getAngularPositionRotations() * INTAKE.gearRatio);
    m_intakeMotor2SimState.setRotorVelocity(
        m_intakeMotor2Sim.getAngularVelocityRPM() * INTAKE.gearRatio / 60.0);
    m_intakeMotor1SimState.setRawRotorPosition(
        m_intakeMotor1Sim.getAngularPositionRotations() * INTAKE.gearRatio);
    m_intakeMotor1SimState.setRotorVelocity(
        m_intakeMotor1Sim.getAngularVelocityRPM() * INTAKE.gearRatio / 60.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    if (!ROBOT.disableLogging) updateLogger();
  }
}
