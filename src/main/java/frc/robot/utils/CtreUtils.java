package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.BASE;
import frc.robot.constants.SWERVE;

public final class CtreUtils {
  /** Initialize Phoenix Server by creating a dummy device.
   *  We do this so that the CANCoders don't get configured before Phoenix Server is up,
   *  which causes issues with encoder offsets not being set/applied properly.
   */
  public static void initPhoenixServer() {
    if(RobotBase.isReal()) {
      TalonFX dummy = new TalonFX(0);
      Timer.delay(5);
      dummy = null;
    }
  }

  public static TalonFXConfiguration generateTurnMotorConfig() {
    TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();

    turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turnMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnMotorConfig.CurrentLimits.SupplyCurrentLimit = 25;
    turnMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    turnMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

    turnMotorConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kTurnMotorGearRatio;
    turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    //    turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    //    turnMotorConfig.Slot0.kV = 0.0;
    turnMotorConfig.Slot0.kP = 100.0;
    turnMotorConfig.Slot0.kI = 0.0;
    //    turnMotorConfig.Slot0.integralZone = 121.904762;
    turnMotorConfig.Slot0.kD = 0.0; // 0.0;
    //    turnMotorConfig.Slot0.allowableClosedloopError = 0.0;

    return turnMotorConfig;
  }

  public static TalonFXConfiguration generateDriveMotorConfig() {
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

    driveMotorConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kDriveMotorGearRatio;
    //    driveMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    //    driveMotorConfig.Slot0.kV = 0.1185;
    driveMotorConfig.Slot0.kP = 0.2402346041055719;
    driveMotorConfig.Slot0.kI = 0.0;
    driveMotorConfig.Slot0.kD = 0.0;

    driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25; // TODO adjust this later
    driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25; // TODO adjust this later

    driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1; // TODO adjust this later
    driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1; // TODO Adjust this later

    return driveMotorConfig;
  }

  public static CANcoderConfiguration generateCanCoderConfig() {
    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();

    sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    sensorConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Unsigned_0To1; // TODO Adjust code for this

    return sensorConfig;
  }

  public static boolean configureTalonFx(TalonFX motor, TalonFXConfiguration config) {
    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      motorStatus = motor.getConfigurator().apply(config);
      if (motorStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    if (!motorStatus.isOK())
      System.out.println(
          "Could not apply configs to TalonFx ID: "
              + motor.getDeviceID()
              + ". Error code: "
              + motorStatus);
    else System.out.println("TalonFX ID: " + motor.getDeviceID() + " - Successfully configured!");
    return motorStatus.isOK();
  }

  public static boolean configureCANCoder(CANcoder cancoder, CANcoderConfiguration config) {
    StatusCode cancoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      cancoderStatus = cancoder.getConfigurator().apply(config);
      if (cancoderStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    if (!cancoderStatus.isOK())
      System.out.println(
          "Could not apply configs to CANCoder ID: "
              + cancoder.getDeviceID()
              + ". Error code: "
              + cancoderStatus);
    else
      System.out.println("CANCoder ID: " + cancoder.getDeviceID() + " - Successfully configured!");
    return cancoderStatus.isOK();
  }
}
