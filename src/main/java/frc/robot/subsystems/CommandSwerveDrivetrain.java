package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.ModuleMap;
import java.io.File;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private final Pose2d[] m_modulePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};
  private final SwerveModuleConstants[] m_constants = new SwerveModuleConstants[4];

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    resetGyro(0);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    System.out.println("Swerve Init at: " + Logger.getRealTimestamp());
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    // for (int i = 0; i < modules.length; i++) {
    //   m_constants[i] = modules[i];
    //   var encoderConfigs = CtreUtils.generateCanCoderConfig();
    //   // encoderConfigs.MagnetSensor.MagnetOffset = modules[i].CANcoderOffset;
    //   CtreUtils.configureCANCoder(getModule(i).getCANcoder(), encoderConfigs);

    //   var turnConfigs = CtreUtils.generateTurnMotorConfig();
    //   // turnConfigs.Feedback.FeedbackRemoteSensorID = modules[i].CANcoderId;
    //   CtreUtils.configureTalonFx(getModule(i).getSteerMotor(), turnConfigs);
    //   setTurnAngle(i, 0);

    //   var driveConfigs = CtreUtils.generateDriveMotorConfig();
    //   // driveConfigs.MotorOutput.Inverted = i % 2 == 0 ? InvertedValue.Clockwise_Positive :
    // InvertedValue.CounterClockwise_Positive;
    //   CtreUtils.configureTalonFx(getModule(i).getDriveMotor(), driveConfigs);
    // }
    resetGyro(0);

    if (Utils.isSimulation()) {
      startSimThread();
    }
    System.out.println("Swerve Init at: " + Logger.getTimestamp());
  }

  public void setTurnAngle(int moduleId, double angle) {
    var newAngle =
        getModule(moduleId).getCANcoder().getAbsolutePosition().getValue()
            - Units.rotationsToDegrees(m_constants[moduleId].CANcoderOffset)
            + angle;

    StatusCode turnMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      turnMotorStatus = getModule(moduleId).getSteerMotor().setPosition(newAngle / 360.0);
      if (turnMotorStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }

    if (!turnMotorStatus.isOK()) {
      System.out.println(
          "Could not update Swerve Turn TalonFX Angle: "
              + getModule(moduleId).getSteerMotor().getDeviceID()
              + ". Error code: "
              + turnMotorStatus);
    } else {
      // System.out.printf(
      //     """
      //                 Updated Turn Motor %2d Angle:
      //                 Desired Angle: %.2f
      //                 Turn Motor Angle: %.2f
      //                 CANCoder Absolute Angle: %.2f
      //                 CANCoder Offset: %.2f\n""",
      //     m_turnMotor.getDeviceID(),
      //     angle,
      //     getTurnHeadingDeg(),
      //     getTurnEncoderAbsHeading().getDegrees(),
      //     m_angleOffset);
    }
  }

  public void resetOdometry(Pose2d pose) {}

  public ChassisSpeeds getChassisSpeed() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
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
    System.out.println("Finished Initializing Drive Settings");
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
    System.out.println("Finished Initializing Drive Settings");
  }

  public void updateLog() {
    Logger.recordOutput("Swerve/Gyro", getPigeon2().getYaw().getValue());
    Logger.recordOutput(
        "Swerve/FRONTLEFTENCODER",
        Units.rotationsToDegrees(getModule(0).getCANcoder().getAbsolutePosition().getValue()));
    Logger.recordOutput(
        "Swerve/FRONTRIGHTENCODER",
        Units.rotationsToDegrees(getModule(1).getCANcoder().getAbsolutePosition().getValue()));
    Logger.recordOutput(
        "Swerve/BACKLEFTENCODER",
        Units.rotationsToDegrees(getModule(2).getCANcoder().getAbsolutePosition().getValue()));
    Logger.recordOutput(
        "Swerve/BACKRIGHTENCODER",
        Units.rotationsToDegrees(getModule(3).getCANcoder().getAbsolutePosition().getValue()));
  }

  @Override
  public void periodic() {
    updateLog();
  }
}
