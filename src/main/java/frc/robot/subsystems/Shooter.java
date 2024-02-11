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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SHOOTER;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private double m_rpm;
  private boolean m_testMode = false;
  private double m_headingOffset;
  private double flywheelRPMRatio = 1.0;
  private double m_desiredPercentOutput;

  private final TalonFX[] m_shooterMotors = {
    new TalonFX(CAN.flywheel1), new TalonFX(CAN.flywheel2) // Flywheel[0] is bottom
  };

  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final VelocityTorqueCurrentFOC m_focControlBottom = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC m_focControlTop = new VelocityTorqueCurrentFOC(0);

  private final SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(SHOOTER.kS, SHOOTER.kV, SHOOTER.kA);
  private SimpleMotorFeedforward m_currentFeedForward = m_feedForward;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Shooter() {
    TalonFXConfiguration configBottom = new TalonFXConfiguration();
    configBottom.Feedback.SensorToMechanismRatio = SHOOTER.gearRatioBottom;
    configBottom.Slot0.kP = SHOOTER.kP;
    configBottom.Slot0.kI = SHOOTER.kI;
    configBottom.Slot0.kD = SHOOTER.kD;
    CtreUtils.configureTalonFx(m_shooterMotors[0], configBottom);

    TalonFXConfiguration configTop = new TalonFXConfiguration();
    configTop.Feedback.SensorToMechanismRatio = SHOOTER.gearRatioTop;
    configTop.Slot0.kV = SHOOTER.kV;
    configTop.Slot0.kP = SHOOTER.kP;
    configTop.Slot0.kI = SHOOTER.kI;
    configTop.Slot0.kD = SHOOTER.kD;
    CtreUtils.configureTalonFx(m_shooterMotors[1], configTop);

    m_shooterMotors[0].setInverted(true);
    // flywheel motor 1
    m_shooterMotors[1].setControl(new Follower(m_shooterMotors[0].getDeviceID(), true));
  }

  // values that we set
  public void setPercentOutput(double percentOutput) {
    m_shooterMotors[0].setControl(m_dutyCycleRequest.withOutput(percentOutput));

    m_desiredPercentOutput = percentOutput;
  }

  public void setRpmOutput(double rpm) {
    // Phoenix 6 uses rotations per second for velocity control
    var rps = rpm / 60.0;
    m_shooterMotors[0].setControl(
        m_focControlTop.withVelocity(rps).withFeedForward(m_currentFeedForward.calculate(rps)));
  }

  public double getShootNStrafeAngle(
      Pose2d robotPose, double RobotVelocityX, double RobotVelocityY) {
    return Math.atan2(
        (SHOOTER.NoteVelocity * Math.sin(getShootAngle(robotPose)) - RobotVelocityY),
        (SHOOTER.NoteVelocity * Math.cos(getShootAngle(robotPose)) - RobotVelocityX));
  }

  public double getShootAngle(Pose2d robotPose) {
    if (Controls.isBlueAllaince()) {
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
  
  // Gets the frictional flywheel velocity in meters per second
  public double getMotorVelocityMetersPerSecond() {
    //return (m_rpm / 60.0) * FLYWHEEL.kFlywheelDiameter * Math.PI;
    return 2;
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
    Logger.recordOutput("Shooter/MasterPercentOutput", m_shooterMotors[0].get());
    Logger.recordOutput("Shooter/FollowerPercentOutput", m_shooterMotors[1].get());
    Logger.recordOutput("Shooter/RPMMaster", getRpmMaster());
    Logger.recordOutput("Shooter/RPMFollower", getRpmFollower());
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    if (!ROBOT.disableLogging) updateLogger();
  }
}
