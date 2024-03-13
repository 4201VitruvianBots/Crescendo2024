// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
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
  private double m_topRpmSetpoint;
  private double m_bottomRpmSetpoint;

  private double m_rpmTop;
  private double m_rpmBottom;
  private boolean m_testMode = false;
  private boolean m_isShooting = false;
  // private double m_headingOffset;
  private double m_desiredPercentOutput;

  private final TalonFX[] m_shooterMotors = {
    new TalonFX(CAN.flywheel1), new TalonFX(CAN.flywheel2) // Flywheel[0] is bottom
  };

  private final DCMotorSim[] m_shooterMotorSim = {
    new DCMotorSim(SHOOTER.ShooterBottomGearbox, SHOOTER.gearRatioBottom, SHOOTER.Inertia),
    new DCMotorSim(SHOOTER.ShooterTopGearbox, SHOOTER.gearRatioTop, SHOOTER.Inertia)
  };

  private final StatusSignal<Double> m_mainVelocitySignal =
      m_shooterMotors[0].getVelocity().clone();
  private final StatusSignal<Double> m_followerVelocitySignal =
      m_shooterMotors[1].getVelocity().clone();
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final TorqueCurrentFOC m_TorqueCurrentFOC = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC m_focVelocityControl = new VelocityTorqueCurrentFOC(0);

  private final TalonFXSimState m_shooterMotorBottomSimState = m_shooterMotors[0].getSimState();
  private final TalonFXSimState m_shooterMotorTopSimState = m_shooterMotors[1].getSimState();

  // This is currently not used
  //   private final SimpleMotorFeedforward m_feedForward =
  //       new SimpleMotorFeedforward(SHOOTER.kS, SHOOTER.kV, SHOOTER.kA);
  //   private SimpleMotorFeedforward m_currentFeedForward = m_feedForward;

  /* Creates a new Intake. */
  public Shooter() {
    TalonFXConfiguration configBottom = new TalonFXConfiguration();
    configBottom.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configBottom.Feedback.SensorToMechanismRatio = SHOOTER.gearRatioBottom;
    configBottom.Slot0.kP = SHOOTER.kPBottom;
    configBottom.Slot0.kI = SHOOTER.kIBottom;
    configBottom.Slot0.kD = SHOOTER.kDBottom;
    configBottom.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    CtreUtils.configureTalonFx(m_shooterMotors[0], configBottom);

    TalonFXConfiguration configTop = new TalonFXConfiguration();
    configTop.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configTop.Feedback.SensorToMechanismRatio = SHOOTER.gearRatioTop;
    configTop.Slot0.kP = SHOOTER.kPTop;
    configTop.Slot0.kI = SHOOTER.kITop;
    configTop.Slot0.kD = SHOOTER.kDTop;
    configTop.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    CtreUtils.configureTalonFx(m_shooterMotors[1], configTop);

    m_shooterMotors[0].setInverted(false);
    m_shooterMotors[1].setInverted(true);
    
    SHOOTER.SPEAKER.initConstants(Controls.isBlueAlliance());
  }

  public boolean getIsShooting() {
    return (m_bottomRpmSetpoint != 0 || m_topRpmSetpoint != 0);
  }

  public void setShootingState(boolean state) {
    m_isShooting = state;
  }

  public boolean getShootingState() {
    return m_isShooting;
  }

  public boolean getReved() {
    if (getShootingState()) {
      return (getRpmFollower() >= 7000 && getRpmMaster() >= 7000);
    } else return false;
  }

  public boolean getUnreved() {
    if (getShootingState()) {
      return (getRpmFollower() < 7000 && getRpmMaster() < 7000);
    } else return false;
  }

  public boolean getZoneState() {
    return true; // TODO: Change this to true only if we are in zone
  }

  // values that we set
  public void setPercentOutput(double percentOutput) {
    m_desiredPercentOutput = percentOutput;
    m_bottomRpmSetpoint = 0;
    m_topRpmSetpoint = 0;

    m_shooterMotors[0].setControl(m_dutyCycleRequest.withOutput(percentOutput));
    m_shooterMotors[1].setControl(m_dutyCycleRequest.withOutput(percentOutput));
  }

  public void setVoltageOutput(double voltageOut) {
    m_bottomRpmSetpoint = 0;
    m_topRpmSetpoint = 0;
    m_shooterMotors[0].setControl(m_voltageRequest.withOutput(voltageOut));
    m_shooterMotors[1].setControl(m_voltageRequest.withOutput(voltageOut));
  }

  public void setFocCurrentOutput(double currentOut) {
    m_bottomRpmSetpoint = 0;
    m_topRpmSetpoint = 0;
    m_shooterMotors[0].setControl(m_TorqueCurrentFOC.withOutput(currentOut));
    m_shooterMotors[1].setControl(m_TorqueCurrentFOC.withOutput(currentOut));
  }

  public void setRPMOutputFOC(double rpm) {
    m_bottomRpmSetpoint = rpm;
    m_topRpmSetpoint = rpm;

    // Phoenix 6 uses rotations per second for velocity control
    var rps = rpm / 60.0;
    m_shooterMotors[0].setControl(m_focVelocityControl.withVelocity(rps).withFeedForward(0));
    m_shooterMotors[1].setControl(m_focVelocityControl.withVelocity(rps).withFeedForward(0));
  }

  public void setRPMOutput(double rpm) {
    // setRPMOutput(rpm, rpm);
    setRPMOutputFOC(rpm);
  }

  public void setRPMOutput(double rpmBottom, double rpmTop) {
    m_bottomRpmSetpoint = rpmBottom;
    m_topRpmSetpoint = rpmTop;
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


  //checks if
  public boolean isValidShotPose(Pose2d robotpose)
  {
      return (
            (((SHOOTER.robotshooterheight - SHOOTER.SPEAKER.SpeakerBottomZ) / Math.tan(SHOOTER.kShooterAngle) 
              < Math.sqrt(
                  Math.pow(robotpose.getX() - SHOOTER.SPEAKER.SpeakerBottomRightX, 2) 
                + Math.pow(robotpose.getY() - SHOOTER.SPEAKER.SpeakerBottomRightY, 2))) 
          && ((SHOOTER.robotshooterheight - SHOOTER.SPEAKER.SpeakerTopZ) / Math.tan(SHOOTER.kShooterAngle) 
              > Math.sqrt(
                  Math.pow(robotpose.getX() - SHOOTER.SPEAKER.SpeakerTopLeftX, 2) 
                + Math.pow(robotpose.getY() - SHOOTER.SPEAKER.SpeakerTopLeftY, 2))))
          || (((SHOOTER.robotshooterheight - SHOOTER.SPEAKER.SpeakerBottomZ) / Math.tan(SHOOTER.kShooterAngle)
              < Math.sqrt(
              Math.pow(robotpose.getX() - SHOOTER.SPEAKER.SpeakerBottomLeftX, 2) 
            + Math.pow(robotpose.getY() - SHOOTER.SPEAKER.SpeakerBottomLeftY, 2)))
          && ((SHOOTER.robotshooterheight - SHOOTER.SPEAKER.SpeakerTopZ) / Math.tan(SHOOTER.kShooterAngle)
              > Math.sqrt(
                  Math.pow(robotpose.getX() - SHOOTER.SPEAKER.SpeakerTopRightX, 2) 
                + Math.pow(robotpose.getY() - SHOOTER.SPEAKER.SpeakerTopRightY, 2)))));
  }

  // values that we are pulling
  // Changed By Sheraz to avoid Loop Overruns
  public double getRpmMaster() {
    m_mainVelocitySignal.refresh();
    return m_mainVelocitySignal.getValue() * 60.0;
  }

  public double getRpmFollower() {
    m_followerVelocitySignal.refresh();
    return m_followerVelocitySignal.getValue() * 60.0;
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
    // m_currentFeedForward = new SimpleMotorFeedforward(s, v, a);
  }

  public boolean getTestMode() {
    return m_testMode;
  }

  public void setTestMode(boolean mode) {
    m_testMode = mode;
  }

  public double getTopRPMsetpoint() {
    return m_topRpmSetpoint;
  }

  public double getBottomRPMsetpoint() {
    return m_bottomRpmSetpoint;
  }

  private void updateShuffleboard() {}

  // values that we are pulling

  private void updateLogger() {
    Logger.recordOutput("Shooter/DesiredPercentOutput", m_desiredPercentOutput);
    Logger.recordOutput(
        "Shooter/MasterPercentOutput", m_shooterMotors[0].getMotorVoltage().getValue() / 12.0);
    Logger.recordOutput(
        "Shooter/FollowerPercentOutput", m_shooterMotors[1].getMotorVoltage().getValue() / 12.0);
    Logger.recordOutput("Shooter/rpmSetpointTop", m_topRpmSetpoint);
    Logger.recordOutput("Shooter/rpmSetpointBottom", m_bottomRpmSetpoint);
    Logger.recordOutput("Shooter/RPMMaster", getRpmMaster());
    Logger.recordOutput("Shooter/RPMFollower", getRpmFollower());
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();
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
