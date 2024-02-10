// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.FLYWHEEL;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private double m_rpm;
  private boolean m_testMode = false;
  private double m_headingOffset;
  private double flywheelRPMRatio = 1.0;
  private double m_percentOutput;

  private final TalonFX[] m_shooterMotors = {
    new TalonFX(CAN.flywheel1), new TalonFX(CAN.flywheel2) // Flywheel[0] is bottom
  };

  private double flywheelPercentRatio = 1.0;
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  private final SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(FLYWHEEL.kS, FLYWHEEL.kV, FLYWHEEL.kA);
  private SimpleMotorFeedforward m_currentFeedForward = m_feedForward;

  VelocityTorqueCurrentFOC FOCcontrolBottom = new VelocityTorqueCurrentFOC(0);

  VelocityTorqueCurrentFOC FOCcontrolTop = new VelocityTorqueCurrentFOC(0);

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Shooter() {
    TalonFXConfiguration configBottom = new TalonFXConfiguration();

    configBottom.Feedback.SensorToMechanismRatio = FLYWHEEL.gearRatioBottom;
    CtreUtils.configureTalonFx(m_shooterMotors[0], configBottom);

    TalonFXConfiguration configTop = new TalonFXConfiguration();
    configBottom.Feedback.SensorToMechanismRatio = FLYWHEEL.gearRatioTop;
    CtreUtils.configureTalonFx(m_shooterMotors[1], configTop);

    configBottom.Slot0.kP = FLYWHEEL.kP;
    configBottom.Slot0.kI = FLYWHEEL.kI;
    configBottom.Slot0.kD = FLYWHEEL.kD;

    configTop.Slot0.kV = FLYWHEEL.kV;
    configTop.Slot0.kP = FLYWHEEL.kP;
    configTop.Slot0.kI = FLYWHEEL.kI;
    configTop.Slot0.kD = FLYWHEEL.kD;

    m_shooterMotors[0].setInverted(true);
    // flywheel motor 1
    m_shooterMotors[1].setControl(new Follower(m_shooterMotors[0].getDeviceID(), true));
  }

  // values that we set
  public void setPercentOutput(double percentOutput) {
    m_shooterMotors[0].setControl(m_dutyCycleRequest.withOutput(percentOutput));

    m_percentOutput = percentOutput;
  }

  public double getPercentOutput() {
    return m_percentOutput;
  }

  public void setRpmOutput(double rpm) {
    // Phoenix 6 uses rotations per second for velocity control

    var rps = rpm / 60.0;
    m_shooterMotors[0].setControl(
        FOCcontrolTop.withVelocity(rps).withFeedForward(m_currentFeedForward.calculate(rps)));
  }

  public double ShootNStrafeAngle(
      double RobotPoseX, double RobotPoseY, double RobotVelocityX, double RobotVelocityY) {
    return Math.atan2(
        (FLYWHEEL.NoteVelocity * Math.sin(this.shootangle(RobotPoseX, RobotPoseY))
            - RobotVelocityY),
        (FLYWHEEL.NoteVelocity * Math.cos(this.shootangle(RobotPoseX, RobotPoseY))
            - RobotVelocityX));
  }

  public double shootangle(Double RobotPoseX, double RobotPoseY) {
    if (Controls.IsBlueAllaince()) {
      return (Math.atan2(
                  (FLYWHEEL.SPEAKER.BlueSpeakerTLY - RobotPoseY),
                  (FLYWHEEL.SPEAKER.BlueSpeakerTLX - RobotPoseX))
              + Math.atan2(
                  (FLYWHEEL.SPEAKER.BlueSpeakerTRY - RobotPoseY),
                  (FLYWHEEL.SPEAKER.BlueSpeakerTRX - RobotPoseX))
              + Math.atan2(
                  (FLYWHEEL.SPEAKER.BlueSpeakerBLY - RobotPoseY),
                  (FLYWHEEL.SPEAKER.BlueSpeakerBLX - RobotPoseX))
              + Math.atan2(
                  (FLYWHEEL.SPEAKER.BlueSpeakerBRY - RobotPoseY),
                  (FLYWHEEL.SPEAKER.BlueSpeakerBRX - RobotPoseX)))
          / 4;
    } else {
      return (Math.atan2(
                  (FLYWHEEL.SPEAKER.RedSpeakerTLY - RobotPoseY),
                  (FLYWHEEL.SPEAKER.RedSpeakerTLX - RobotPoseX))
              + Math.atan2(
                  (FLYWHEEL.SPEAKER.RedSpeakerTRY - RobotPoseY),
                  (FLYWHEEL.SPEAKER.RedSpeakerTRX - RobotPoseX))
              + Math.atan2(
                  (FLYWHEEL.SPEAKER.RedSpeakerBLY - RobotPoseY),
                  (FLYWHEEL.SPEAKER.RedSpeakerBLX - RobotPoseX))
              + Math.atan2(
                  (FLYWHEEL.SPEAKER.RedSpeakerBRY - RobotPoseY),
                  (FLYWHEEL.SPEAKER.RedSpeakerBRX - RobotPoseX)))
          / 4;
    }
  }

  // values that we are pulling
  public double getRPMMaster() {
    return m_shooterMotors[0].getVelocity().getValueAsDouble() * 60.0;
  }

  public double getRPMFollower() {
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

  public void setTestMode(boolean mode) {
    m_testMode = mode;
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Flywheel/PercentOutputPredicted", m_percentOutput);
    SmartDashboard.putNumber(
        "Flywheel/MasterPercentOutputActual",
        m_shooterMotors[0].getVelocity().getValueAsDouble() / 51.1998046875 / 2);
    SmartDashboard.putNumber(
        "Flywheel/FollowerPercentOutputActual",
        m_shooterMotors[1].getVelocity().getValueAsDouble() / 51.1998046875 / 2);
    SmartDashboard.putNumber("Flywheel/RPMMaster", getRPMMaster());
    SmartDashboard.putNumber("Flywheel/RPMFollower", getRPMFollower());
  }

  // values that we are pulling

  private void updateLogger() {
    Logger.recordOutput("Flywheel/PercentOutputPredicted", getPercentOutput());
    Logger.recordOutput(
        "Flywheel/MasterPercentOutputActual",
        m_shooterMotors[0].getVelocity().getValueAsDouble() / 512);
    Logger.recordOutput(
        "Flywheel/FollowerPercentOutputActual",
        m_shooterMotors[1].getVelocity().getValueAsDouble() / 512);
    Logger.recordOutput("Flywheel/RPMMaster", getRPMMaster());
    Logger.recordOutput("Flywheel/RPMFollower", getRPMFollower());
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    updateLogger();
  }
}
