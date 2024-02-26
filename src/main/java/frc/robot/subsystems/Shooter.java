// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SHOOTER;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private double m_rpm;
  private double m_rpmTop;
  private double m_rpmBottom;
  private boolean m_testMode = false;
  private double m_headingOffset;
  private double m_desiredPercentOutput;
  private boolean isShooting = false;

  private final TalonFX[] m_shooterMotors = {
    new TalonFX(CAN.flywheel1), new TalonFX(CAN.flywheel2) // Flywheel[0] is bottom
  };

  private final DCMotorSim[] m_shooterMotorSim = {
    new DCMotorSim(SHOOTER.ShooterBottomGearbox, SHOOTER.gearRatioBottom, SHOOTER.Inertia),
    new DCMotorSim(SHOOTER.ShooterTopGearbox, SHOOTER.gearRatioTop, SHOOTER.Inertia)
  };

  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final TorqueCurrentFOC m_TorqueCurrentFOC = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC m_focVelocityControl = new VelocityTorqueCurrentFOC(0);

  private final TalonFXSimState m_shooterMotorBottomSimState = m_shooterMotors[0].getSimState();
  private final TalonFXSimState m_shooterMotorTopSimState = m_shooterMotors[1].getSimState();

  private final SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(SHOOTER.kS, SHOOTER.kV, SHOOTER.kA);
  private SimpleMotorFeedforward m_currentFeedForward = m_feedForward;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Shooter() {
    TalonFXConfiguration configBottom = new TalonFXConfiguration();
    configBottom.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configBottom.Feedback.SensorToMechanismRatio = SHOOTER.gearRatioBottom;
    configBottom.Slot0.kP = SHOOTER.bottomkP;
    configBottom.Slot0.kI = SHOOTER.bottomkI;
    configBottom.Slot0.kD = SHOOTER.bottomkD;
    configBottom.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    CtreUtils.configureTalonFx(m_shooterMotors[0], configBottom);

    TalonFXConfiguration configTop = new TalonFXConfiguration();
    configTop.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configTop.Feedback.SensorToMechanismRatio = SHOOTER.gearRatioTop;
    configTop.Slot0.kP = SHOOTER.topkP;
    configTop.Slot0.kI = SHOOTER.topkI;
    configTop.Slot0.kD = SHOOTER.topkD;
    configTop.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    CtreUtils.configureTalonFx(m_shooterMotors[1], configTop);

    m_shooterMotors[0].setInverted(false);
    m_shooterMotors[1].setInverted(true);
  }

  public boolean getShooterState() {
    return isShooting;
  }

  /** Sets a boolean for the intake's actuation */
  public void setShooterState(boolean state) {
    isShooting = state;
  }

  // values that we set
  public void setPercentOutput(double percentOutput) {
    m_desiredPercentOutput = percentOutput;

    m_shooterMotors[0].setControl(m_dutyCycleRequest.withOutput(percentOutput));
    m_shooterMotors[1].setControl(m_dutyCycleRequest.withOutput(percentOutput));
  }

  public void setVoltageOutput(double voltageOut) {
    m_shooterMotors[0].setControl(m_voltageRequest.withOutput(voltageOut));
    m_shooterMotors[1].setControl(m_voltageRequest.withOutput(voltageOut));
  }

  public void setFocCurrentOutput(double currentOut) {
    m_shooterMotors[0].setControl(m_TorqueCurrentFOC.withOutput(currentOut));
    m_shooterMotors[1].setControl(m_TorqueCurrentFOC.withOutput(currentOut));
  }

  public void setRPMOutputFOC(double rpm) {
    m_rpm = rpm;

    // Phoenix 6 uses rotations per second for velocity control
    var rps = rpm / 60.0;
    m_shooterMotors[0].setControl(m_focVelocityControl.withVelocity(rps).withFeedForward(0));
    m_shooterMotors[1].setControl(m_focVelocityControl.withVelocity(rps).withFeedForward(0));
  }

  public void setRPMOutput(double rpm) {
    setRPMOutput(rpm, rpm);
  }

  public void setRPMOutput(double rpmBottom, double rpmTop) {
    m_rpmBottom = rpmBottom;
    m_rpmTop = rpmTop;
    // Phoenix 6 uses rotations per second for velocity control
    var rpsBottom = rpmBottom / 60.0;
    var rpsTop = rpmTop / 60.0;
    m_shooterMotors[0].setControl(m_velocityRequest.withVelocity(rpsBottom).withFeedForward(0));
    m_shooterMotors[1].setControl(m_velocityRequest.withVelocity(rpsTop).withFeedForward(0));
  }

  public void setNeutralMode(NeutralModeValue mode) {
    m_shooterMotors[0].setNeutralMode(mode);
    m_shooterMotors[1].setNeutralMode(mode);
  }

  public double getShootNStrafeAngle(
      Pose2d robotPose, double RobotVelocityX, double RobotVelocityY) {
    return Math.atan2(
        (SHOOTER.NoteVelocity * Math.sin(getShootAngle(robotPose)) - RobotVelocityY),
        (SHOOTER.NoteVelocity * Math.cos(getShootAngle(robotPose)) - RobotVelocityX));
  }

  public double getShootAngle(Pose2d robotPose) {
    if (Controls.isBlueAlliance()) {
      return (Math.atan2(
                  (SHOOTER.SPEAKER.BlueSpeakerTLY - robotPose.getY()),
                  (SHOOTER.SPEAKER.BlueSpeakerTLX - robotPose.getX()))
              + Math.atan2(
                  (SHOOTER.SPEAKER.BlueSpeakerTRY - robotPose.getY()),
                  (SHOOTER.SPEAKER.BlueSpeakerTRX - robotPose.getX()))
              + Math.atan2(
                  (SHOOTER.SPEAKER.BlueSpeakerBLY - robotPose.getY()),
                  (SHOOTER.SPEAKER.BlueSpeakerBLX - robotPose.getX()))
              + Math.atan2(
                  (SHOOTER.SPEAKER.BlueSpeakerBRY - robotPose.getY()),
                  (SHOOTER.SPEAKER.BlueSpeakerBRX - robotPose.getX())))
          / 4;
    } else {
      return (Math.atan2(
                  (SHOOTER.SPEAKER.RedSpeakerTLY - robotPose.getY()),
                  (SHOOTER.SPEAKER.RedSpeakerTLX - robotPose.getX()))
              + Math.atan2(
                  (SHOOTER.SPEAKER.RedSpeakerTRY - robotPose.getY()),
                  (SHOOTER.SPEAKER.RedSpeakerTRX - robotPose.getX()))
              + Math.atan2(
                  (SHOOTER.SPEAKER.RedSpeakerBLY - robotPose.getY()),
                  (SHOOTER.SPEAKER.RedSpeakerBLX - robotPose.getX()))
              + Math.atan2(
                  (SHOOTER.SPEAKER.RedSpeakerBRY - robotPose.getY()),
                  (SHOOTER.SPEAKER.RedSpeakerBRX - robotPose.getX())))
          / 4;
    }
  }

  // values that we are pulling
  public double getRpmMaster() {
    return m_shooterMotors[0].getVelocity().getValueAsDouble() * 60.0;
  }

  public double getRpmFollower() {
    return m_shooterMotors[1].getVelocity().getValueAsDouble() * 60.0;
  }

  public void setPidValues(double v, double p, double i, double d) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Get the current motor configs to not erase everything
    m_shooterMotors[0].getConfigurator().refresh(config);
    config.Slot0.kV = v;
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    CtreUtils.configureTalonFx(m_shooterMotors[0], config);
  }

  public void setSimpleMotorFeedForward(double s, double v, double a) {
    m_currentFeedForward = new SimpleMotorFeedforward(s, v, a);
  }

  public boolean getTestMode() {
    return m_testMode;
  }

  public void setTestMode(boolean mode) {
    m_testMode = mode;
  }

  private void updateShuffleboard() {}

  // values that we are pulling

  private void updateLogger() {
    Logger.recordOutput("Shooter/DesiredPercentOutput", m_desiredPercentOutput);
    Logger.recordOutput(
        "Shooter/MasterPercentOutput", m_shooterMotors[0].getMotorVoltage().getValue() / 12.0);
    Logger.recordOutput(
        "Shooter/FollowerPercentOutput", m_shooterMotors[1].getMotorVoltage().getValue() / 12.0);
    Logger.recordOutput("Shooter/rpmsetpoint", m_rpm);
    Logger.recordOutput("Shooter/rpmsetpointTop", m_rpmTop);
    Logger.recordOutput("Shooter/rpmsetpointBottom", m_rpmBottom);
    Logger.recordOutput("Shooter/RPMMaster", getRpmMaster());
    Logger.recordOutput("Shooter/RPMFollower", getRpmFollower());
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    if (!ROBOT.disableLogging) updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    m_shooterMotorTopSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_shooterMotorBottomSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_shooterMotorSim[1].setInputVoltage(
        MathUtil.clamp(m_shooterMotorTopSimState.getMotorVoltage(), -12, 12));
    m_shooterMotorSim[0].setInputVoltage(
        MathUtil.clamp(m_shooterMotorBottomSimState.getMotorVoltage(), -12, 12));

    m_shooterMotorSim[1].update(RobotTime.getTimeDelta());
    m_shooterMotorSim[0].update(RobotTime.getTimeDelta());

    m_shooterMotorTopSimState.setRawRotorPosition(
        m_shooterMotorSim[1].getAngularPositionRotations() * SHOOTER.gearRatioTop);
    m_shooterMotorTopSimState.setRotorVelocity(
        m_shooterMotorSim[1].getAngularVelocityRPM() * SHOOTER.gearRatioTop / 60.0);
    m_shooterMotorBottomSimState.setRawRotorPosition(
        m_shooterMotorSim[0].getAngularPositionRotations() * SHOOTER.gearRatioBottom);
    m_shooterMotorBottomSimState.setRotorVelocity(
        m_shooterMotorSim[0].getAngularVelocityRPM() * SHOOTER.gearRatioBottom / 60.0);
  }
}
