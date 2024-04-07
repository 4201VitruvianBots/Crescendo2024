package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.FIELD;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SWERVE;
import frc.robot.constants.VISION;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.ModuleMap;
import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private Vision m_vision;
  private final Alert m_alert = new Alert("SwerveDrivetrain", AlertType.INFO);

  private final SwerveRequest.FieldCentric m_driveReqeustFieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
          .withRotationalDeadband(
              SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.RobotCentric m_driveReqeustRobotCentric =
      new SwerveRequest.RobotCentric()
          .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
          .withRotationalDeadband(
              SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.FieldCentricFacingAngle m_turnRequest =
      new SwerveRequest.FieldCentricFacingAngle();
  // driving in open loop
  private Pose2d m_futurePose = new Pose2d();
  private Twist2d m_twistFromPose = new Twist2d();
  private ChassisSpeeds m_newChassisSpeeds = new ChassisSpeeds();

  private final SwerveRequest.ApplyChassisSpeeds m_chassisSpeedRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private final PIDController m_pidController =
      new PIDController(
          SWERVE.DRIVE.kTeleP_Theta, SWERVE.DRIVE.kTeleI_Theta, SWERVE.DRIVE.kAutoD_Theta);
  private Rotation2d m_targetAngle = new Rotation2d();
  private Rotation2d m_angleToSpeaker = new Rotation2d();
  private Rotation2d m_angleToNote = new Rotation2d();
  private VISION.TRACKING_STATE m_trackingState = VISION.TRACKING_STATE.NONE;

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    m_pidController.setTolerance(Units.degreesToRadians(2));
    m_pidController.enableContinuousInput(-Math.PI, Math.PI);
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    if (Utils.isSimulation()) {
      startSimThread();
    }
    m_alert.setText("Swerve Init at: " + Logger.getRealTimestamp());
    m_alert.set(true);
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    m_pidController.setTolerance(Units.degreesToRadians(2));
    m_pidController.enableContinuousInput(-Math.PI, Math.PI);
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    if (Utils.isSimulation()) {
      startSimThread();
    }
    m_alert.setText("Swerve Init at: " + Logger.getTimestamp() * 1.0e-6);
    m_alert.set(true);
  }

  public void registerVisionSubsystem(Vision vision) {
    m_vision = vision;
  }

  public ChassisSpeeds getChassisSpeed() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public Command turnInPlace(Rotation2d angle, BooleanSupplier flipAngle) {
    if (flipAngle.getAsBoolean()) {
      angle = new Rotation2d(Math.PI).minus(angle);
    }

    // variables passed into lambdas must be final
    Rotation2d finalAngle = angle;
    return applyRequest(() -> m_turnRequest.withTargetDirection(finalAngle))
        .onlyWhile(
            () ->
                Math.abs(
                        finalAngle
                            .minus(Rotation2d.fromDegrees(getPigeon2().getYaw().getValue()))
                            .getDegrees())
                    > 1.0)
        .withTimeout(0.25);
  }

  public Command applyChassisSpeeds(Supplier<ChassisSpeeds> chassisSpeeds) {
    return applyChassisSpeeds(chassisSpeeds, 0.02, 1.0, false);
  }

  public Command applyChassisSpeeds(Supplier<ChassisSpeeds> chassisSpeeds, boolean isRobotCentric) {
    return applyChassisSpeeds(chassisSpeeds, 0.02, 1.0, isRobotCentric);
  }

  public Command applyChassisSpeeds(
      Supplier<ChassisSpeeds> chassisSpeeds, double loopPeriod, boolean isRobotCentric) {
    return applyChassisSpeeds(chassisSpeeds, loopPeriod, 1.0, isRobotCentric);
  }

  /**
   * Second-Order Kinematics <a
   * href="https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/79">...</a>
   */
  public Command applyChassisSpeeds(
      Supplier<ChassisSpeeds> chassisSpeeds,
      double loopPeriod,
      double driftRate,
      boolean isRobotCentric) {
    return applyRequest(
        () -> {
          m_futurePose =
              new Pose2d(
                  chassisSpeeds.get().vxMetersPerSecond * loopPeriod,
                  chassisSpeeds.get().vyMetersPerSecond * loopPeriod,
                  Rotation2d.fromRadians(
                      chassisSpeeds.get().omegaRadiansPerSecond * loopPeriod * driftRate));

          m_twistFromPose = new Pose2d().log(m_futurePose);

          var rotationSpeed = chassisSpeeds.get().omegaRadiansPerSecond;
          if (m_trackingState != VISION.TRACKING_STATE.NONE) {
            if (m_trackingState == VISION.TRACKING_STATE.NOTE) {
              if (m_vision != null) {
                if (m_vision.hasGamePieceTarget()) {
                  rotationSpeed = calculateRotationToTarget();
                }
              }
            } else if (m_trackingState == VISION.TRACKING_STATE.SPEAKER) {
              rotationSpeed = calculateRotationToTarget();
            }
          }

          m_newChassisSpeeds =
              new ChassisSpeeds(
                  m_twistFromPose.dx / loopPeriod, m_twistFromPose.dy / loopPeriod, rotationSpeed);

          if (isRobotCentric) {
            return m_driveReqeustRobotCentric
                .withVelocityX(m_newChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(m_newChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(m_newChassisSpeeds.omegaRadiansPerSecond);
          } else {
            if (Controls.isRedAlliance()) {
              return m_driveReqeustFieldCentric
                  .withVelocityX(-m_newChassisSpeeds.vxMetersPerSecond)
                  .withVelocityY(-m_newChassisSpeeds.vyMetersPerSecond)
                  .withRotationalRate(m_newChassisSpeeds.omegaRadiansPerSecond);
            } else {
              return m_driveReqeustFieldCentric
                  .withVelocityX(m_newChassisSpeeds.vxMetersPerSecond)
                  .withVelocityY(m_newChassisSpeeds.vyMetersPerSecond)
                  .withRotationalRate(m_newChassisSpeeds.omegaRadiansPerSecond);
            }
          }
        });
  }

  public void setChassisSpeedControl(ChassisSpeeds chassisSpeeds) {
    setChassisSpeedControl(chassisSpeeds, 0.02, 1.0);
  }

  public void setChassisSpeedControl(ChassisSpeeds chassisSpeeds, double loopPeriod) {
    setChassisSpeedControl(chassisSpeeds, loopPeriod, 1.0);
  }

  /**
   * Second-Order Kinematics <a
   * href="https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/79">...</a>
   */
  public void setChassisSpeedControl(
      ChassisSpeeds chassisSpeeds, double loopPeriod, double driftRate) {
    m_futurePose =
        new Pose2d(
            chassisSpeeds.vxMetersPerSecond * loopPeriod,
            chassisSpeeds.vyMetersPerSecond * loopPeriod,
            Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * loopPeriod * driftRate));

    m_twistFromPose = new Pose2d().log(m_futurePose);

    m_newChassisSpeeds =
        new ChassisSpeeds(
            m_twistFromPose.dx / loopPeriod,
            m_twistFromPose.dy / loopPeriod,
            chassisSpeeds.omegaRadiansPerSecond);

    setControl(m_chassisSpeedRequest.withSpeeds(m_newChassisSpeeds));
  }

  public void setChassisSpeedControlNormal(ChassisSpeeds chassisSpeeds) {
    setControl(m_chassisSpeedRequest.withSpeeds(chassisSpeeds));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    var cmd = run(() -> setControl(requestSupplier.get()));
    cmd.addRequirements(this);
    return cmd;
  }

  public void setAngleToSpeaker(Rotation2d angle) {
    m_angleToSpeaker = angle;
  }

  public void setAngleToNote(Rotation2d angle) {
    m_angleToNote = angle;
  }

  public void setTrackingState(VISION.TRACKING_STATE state) {
    if (m_trackingState != state) {
      m_pidController.reset();
      m_trackingState = state;
    }
  }

  private double calculateRotationToTarget() {
    return m_pidController.calculate(
        getState().Pose.getRotation().getRadians(), m_targetAngle.getRadians());
  }

  private void updateTargetAngle() {
    switch (m_trackingState) {
      case SPEAKER:
        m_targetAngle = m_angleToSpeaker;
        break;
      case NOTE:
        m_targetAngle = m_angleToNote;
        break;
    }
  }

  public boolean isTrackingState() {
    if (m_trackingState == TRACKING_STATE.SPEAKER) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getZoneState() {
    Translation2d m_goal = Controls.isRedAlliance() ? FIELD.redSpeaker : FIELD.blueSpeaker;
    Translation2d robotPose = getState().Pose.getTranslation();

    double toGoalDistance = m_goal.minus(robotPose).getDistance(new Translation2d());

    if ((toGoalDistance <= 3.5) && (toGoalDistance >= 1.5)) {
      return true;
    } else return false;
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    // Some condition that should decide if we want to override rotation
    if (m_vision != null) {
      if (m_trackingState != VISION.TRACKING_STATE.NONE) {
        // Return an optional containing the rotation override (this should be a field relative
        // rotation)
        return Optional.of(m_targetAngle);
      }
    }
    // return an empty optional when we don't want to override the path's rotation
    return Optional.empty();
  }

  public void initDriveSysid() {
    for (ModuleMap.MODULE_POSITION i : ModuleMap.MODULE_POSITION.values()) {
      var driveMotor = getModule(i.ordinal()).getDriveMotor();
      var turnMotor = getModule(i.ordinal()).getSteerMotor();
      CtreUtils.configureTalonFx(driveMotor, new TalonFXConfiguration());
      CtreUtils.configureTalonFx(turnMotor, CtreUtils.generateTurnMotorConfig());
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      BaseStatusSignal.setUpdateFrequencyForAll(
          250, driveMotor.getPosition(), driveMotor.getVelocity(), driveMotor.getMotorVoltage());

      driveMotor.optimizeBusUtilization();
    }

    var signalLoggerDir = new File("/home/lvuser/logger/sysid/");
    if (!signalLoggerDir.exists()) {
      var result = signalLoggerDir.mkdirs();
      System.out.println("mkdirs() result: " + result);
    }

    SignalLogger.setPath(signalLoggerDir.getAbsolutePath());
    m_alert.setText("Finished Initializing Drive Settings");
    m_alert.set(true);
  }

  public void resetGyro(double angle) {
    getPigeon2().setYaw(angle);
  }

  public void initTurnSysid() {
    var turnMotor = getModule(0).getSteerMotor();
    CtreUtils.configureTalonFx(turnMotor, new TalonFXConfiguration());
    turnMotor.setNeutralMode(NeutralModeValue.Brake);
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, turnMotor.getPosition(), turnMotor.getVelocity(), turnMotor.getMotorVoltage());

    turnMotor.optimizeBusUtilization();

    var signalLoggerDir = new File("/home/lvuser/logger/sysid/");
    if (!signalLoggerDir.exists()) {
      var result = signalLoggerDir.mkdirs();
      System.out.println("mkdirs() result: " + result);
    }

    SignalLogger.setPath(signalLoggerDir.getAbsolutePath());
    m_alert.setText("Finished Initializing Drive Settings");
    m_alert.set(true);
  }

  private void updateLogger() {
    Logger.recordOutput("Swerve/TrackingState", m_trackingState);
    Logger.recordOutput("Swerve/TargetAngle", m_targetAngle.getDegrees());
    Logger.recordOutput("Swerve/isShootZone", getZoneState());

    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.DEBUG.get()) {
      Logger.recordOutput("Swerve/Gyro", getPigeon2().getYaw().getValue());
      Logger.recordOutput(
          "Swerve/FrontLeftEncoder",
          Units.rotationsToDegrees(getModule(0).getCANcoder().getAbsolutePosition().getValue()));
      Logger.recordOutput(
          "Swerve/FrontRightEncoder",
          Units.rotationsToDegrees(getModule(1).getCANcoder().getAbsolutePosition().getValue()));
      Logger.recordOutput(
          "Swerve/BackLeftEncoder",
          Units.rotationsToDegrees(getModule(2).getCANcoder().getAbsolutePosition().getValue()));
      Logger.recordOutput(
          "Swerve/BackRightEncoder",
          Units.rotationsToDegrees(getModule(3).getCANcoder().getAbsolutePosition().getValue()));
    }
  }

  @Override
  public void periodic() {
    updateTargetAngle();
    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
