package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class CLIMBER {
    public static final int climbMotor1 = 31;
    public static final int climbMotor2 = 32;

    // //sensors and encoders are tbd
    public static final int kFalconSensorUnitsPerRotation = 2048;
    // public static final int kCANCoderSensorUnitsPerRotation = 4096;

    //values to change
    public static final double upperLimitMeters = (Units.inchesToMeters(0.0));
    public static final double lowerLimitMeters = (Units.inchesToMeters(0.0));

    // public static final DCMotor gearbox = DCMotor.getFalcon500(2);
    public static final double gearRatio = (1 / 26.4);
    // public static final double massKg = 4.0; will be under 125lbs
    public static final double drumRadiusMeters = Units.inchesToMeters(1.185);
    // public static final double centerOffset = Units.inchesToMeters(14);
    // public static final int mech2dAngleDegrees = 35;
    // public static final double kMaxReverseOutput = -0.45;

    // // PID
    public static final double climberHeightSlowdown = 0.75;

    public static final double climberEncoderSlowdown =
        climberHeightSlowdown * kFalconSensorUnitsPerRotation * gearRatio;

    // public static final double kMaxVel = Units.inchesToMeters(1000);
    // public static final double kMaxAccel = Units.inchesToMeters(1800);
    // public static final int kSlotIdx = 0;
    // public static final int kPIDLoopIdx = 0;
    // public static final int kTimeoutMs = 0;

    public static final double encoderCountsToMeters =
        (drumRadiusMeters * 2 * Math.PI) / (kFalconSensorUnitsPerRotation * gearRatio);

    // public static final double kG = 0.02;
    // public static final double kV = 20.0; // 12.57;
    // public static final double kA = 0.02; // 0.04;

    // public static final double kP = 0.06;
    // public static final double kI = 0.00;
    // public static final double kD = 0.00;

    // public static final double kPercentOutputMultiplier = 0.2;
    // public static final double kLimitedPercentOutputMultiplier = 0.1;

    public enum CLIMBSTATE {
      STILL,
      MOVING
    }
  }
