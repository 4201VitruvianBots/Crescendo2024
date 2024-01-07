package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class CtreUtils {
  public static TalonFXConfiguration generateTurnMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfig.Slot0.kV = 0.0;
    motorConfig.Slot0.kP = 5.0;
    motorConfig.Slot0.kI = 0.0000;
    //    motorConfig.Slot0.integralZone = 121.904762;
    motorConfig.Slot0.kD = 0.000; // 0.0;
    //    motorConfig.Slot0.allowableClosedloopError = 0.0;

    motorConfig.Voltage.PeakForwardVoltage = 12;
    motorConfig.Voltage.PeakReverseVoltage = -12;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 25;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    return motorConfig;
  }

  public static TalonFXConfiguration generateDriveMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfig.Slot0.kV = 0.1185;
    motorConfig.Slot0.kP = 0.24;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;

    motorConfig.Voltage.PeakForwardVoltage = 12;
    motorConfig.Voltage.PeakReverseVoltage = -12;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25; // TODO adjust this later
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1; // TODO Adjust this later

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    return motorConfig;
  }

  public static CANcoderConfiguration generateCanCoderConfig() {
    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();

    sensorConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Unsigned_0To1; // TODO Adjust code for this
    sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    return sensorConfig;
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? targetAngle - 180 : targetAngle + 180;
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
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
