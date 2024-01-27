package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.SWERVE;
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
  private Pose2d[] m_modulePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    var pigeonConfig = new Pigeon2Configuration();
    pigeonConfig.MountPose = new MountPoseConfigs().withMountPoseRoll(-180);
    getPigeon2().getConfigurator().apply(pigeonConfig);
  }

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

  public void resetGyro() {
    getPigeon2().setYaw(0);
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
    Logger.recordOutput("Swerve/Gyro", getPigeon2().getAngle());
  }

  @Override
  public void periodic() {
    updateLog();
  }
}
