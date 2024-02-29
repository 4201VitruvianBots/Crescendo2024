package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.CAN;
import frc.robot.constants.SWERVE;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public final class CtreUtils {
  /**
   * Initialize Phoenix Server by creating a dummy device. We do this so that the CANCoders don't
   * get configured before Phoenix Server is up, which causes issues with encoder offsets not being
   * set/applied properly.
   */
  public static void initPhoenixServer() {
    var alert =
        new Alert("Starting Phoenix Server at: " + Logger.getTimestamp() * 1.0e-6, AlertType.INFO);
    alert.set(true);
    if (RobotBase.isReal()) {
      TalonFX dummy = new TalonFX(0, CAN.drivebaseCanbus);
      Timer.delay(5);
      dummy.close();
      dummy = null;
    }
    alert.setText("Phoenix Server finished Init at: " + Logger.getTimestamp() * 1.0e-6);
  }

  public static TalonFXConfiguration generateTurnMotorConfig() {
    TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();

    turnMotorConfig.CustomParams.CustomParam0 = 1; // Identify the config

    turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turnMotorConfig.Slot0 = SWERVE.MODULE.turnGains;
    turnMotorConfig.CurrentLimits.SupplyCurrentLimit = 25;
    turnMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    turnMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    turnMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        100.0 / SWERVE.MODULE.kTurnMotorGearRatio;
    turnMotorConfig.MotionMagic.MotionMagicAcceleration =
        turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * SWERVE.MODULE.kTurnMotorGearRatio;
    turnMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;

    turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    //    turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //    turnMotorConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kTurnMotorGearRatio;
    turnMotorConfig.Feedback.RotorToSensorRatio = SWERVE.MODULE.kTurnMotorGearRatio;
    //    turnMotorConfig.Feedback.FeedbackRemoteSensorID = 0;

    return turnMotorConfig;
  }

  public static TalonFXConfiguration generateDriveMotorConfig() {
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    driveMotorConfig.CustomParams.CustomParam0 = 2; // Identify the config

    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    driveMotorConfig.Slot0 = SWERVE.MODULE.driveGains;
    driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = SWERVE.MODULE.kSlipCurrent;
    driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SWERVE.MODULE.kSlipCurrent;
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = SWERVE.MODULE.kSlipCurrent;
    driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveMotorConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kDriveMotorGearRatio;

    driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25; // TODO adjust this later
    driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25; // TODO adjust this later

    driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1; // TODO adjust this later
    driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1; // TODO Adjust this later

    return driveMotorConfig;
  }

  public static CANcoderConfiguration generateCanCoderConfig() {
    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();

    //    sensorConfig.MagnetSensor.SensorDirection =
    // SensorDirectionValue.CounterClockwise_Positive;
    //    sensorConfig.MagnetSensor.AbsoluteSensorRange =
    //        AbsoluteSensorRangeValue.Unsigned_0To1; // TODO Adjust code for this

    return sensorConfig;
  }

  public static boolean configureTalonFx(TalonFX motor, SwerveModuleConstants constants) {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    int deviceType = motor.getDeviceID() % 2; // 1 == Turn, 0 == Drive;
    if (deviceType == 0) {
      motorConfig.Slot0 = constants.DriveMotorGains;
      motorConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
      motorConfig.MotorOutput.Inverted =
          constants.DriveMotorInverted
              ? InvertedValue.CounterClockwise_Positive
              : InvertedValue.Clockwise_Positive;
    } else if (deviceType == 1) {
      motorConfig.Slot0 = constants.SteerMotorGains;
      motorConfig.Feedback.SensorToMechanismRatio = constants.SteerMotorGearRatio;
      motorConfig.Feedback.RotorToSensorRatio = constants.CouplingGearRatio;
      motorConfig.MotorOutput.Inverted =
          constants.SteerMotorInverted
              ? InvertedValue.CounterClockwise_Positive
              : InvertedValue.Clockwise_Positive;
    }

    return configureTalonFx(motor, motorConfig);
  }

  public static boolean configureTalonFx(TalonFX motor, TalonFXConfiguration config) {
    if (20 <= motor.getDeviceID() || motor.getDeviceID() <= 27) {
      checkSwerveConfigs(motor, config);
    }

    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      motorStatus = motor.getConfigurator().apply(config);
      if (motorStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    if (!motorStatus.isOK()) {
      var alert =
          new Alert(
              "Could not apply configs to TalonFx ID: "
                  + motor.getDeviceID()
                  + ". Error code: "
                  + motorStatus,
              AlertType.ERROR);
      alert.set(true);
    } else System.out.println("TalonFX ID: " + motor.getDeviceID() + " - Successfully configured!");

    return motorStatus.isOK();
  }

  private static void checkSwerveConfigs(TalonFX motor, TalonFXConfiguration config) {
    int deviceType = motor.getDeviceID() % 2; // 1 == Turn, 0 == Drive;

    if (deviceType == 0) {
      if (config.CustomParams.CustomParam0 != 2 && config.CustomParams.CustomParam0 != 0)
        throw new IllegalArgumentException(
            "Attempting to configure Drive Motor with Turn Configs!");
    } else if (deviceType == 1) {
      if (config.CustomParams.CustomParam0 != 1 && config.CustomParams.CustomParam0 != 0)
        throw new IllegalArgumentException(
            "Attempting to configure Turn Motor with Drive Configs!");
    }
  }

  public static boolean configureCANCoder(CANcoder cancoder, CANcoderConfiguration config) {
    StatusCode cancoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      cancoderStatus = cancoder.getConfigurator().apply(config);
      if (cancoderStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    if (!cancoderStatus.isOK()) {
      var alert =
          new Alert(
              "Could not apply configs to CANCoder ID: "
                  + cancoder.getDeviceID()
                  + ". Error code: "
                  + cancoderStatus,
              AlertType.ERROR);
      alert.set(true);
    } else
      System.out.println("CANCoder ID: " + cancoder.getDeviceID() + " - Successfully configured!");
    return cancoderStatus.isOK();
  }
}
