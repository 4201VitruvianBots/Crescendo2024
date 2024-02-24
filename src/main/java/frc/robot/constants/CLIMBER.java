package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class CLIMBER {

  public static final double kDistanceFromIntake = Units.inchesToMeters(17);

  public static final double kPostHeight = Units.inchesToMeters(22);
  public static final double kHookHeight = Units.inchesToMeters(10);
  public static final double kHookLength = Units.inchesToMeters(6);

  public static final double upperLimitMeters = Units.inchesToMeters(21.50);
  public static final double lowerLimitMeters = Units.inchesToMeters(0.0);

  public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
  public static final double gearRatio = 594.0 / 25.0;
  public static final double sprocketRadiusMeters = Units.inchesToMeters(1.185);
  public static final double sprocketRotationsToMeters = sprocketRadiusMeters * 2 * Math.PI;
  // public static final double centerOffset = Units.inchesToMeters(14);
  // public static final double kMaxReverseOutput = -0.45;

  // PID
  public static final double kMaxVel = Units.inchesToMeters(10);
  public static final double kMaxAccel = Units.inchesToMeters(18);
  // public static final int kSlotIdx = 0;
  // public static final int kPIDLoopIdx = 0;
  // public static final int kTimeoutMs = 0;

  public static final double kG = 0.0;
  public static final double kV = 0.0; // 12.57;
  public static final double kA = 0.0; // 0.04;

  public static final double kP = 10.;
  public static final double kI = 0.00;
  public static final double kD = 0.00;

  public static final double kPercentOutputMultiplier = 1.0;
  public static final double kLimitedPercentOutputMultiplier = 0.5;

  public enum CLIMBER_SETPOINT {
    FULL_RETRACT(Units.inchesToMeters(0.0)),
    EXTEND(Units.inchesToMeters(21.25));

    private final double setpointMeters;

    CLIMBER_SETPOINT(double setpointMeters) {
      this.setpointMeters = setpointMeters;
    }

    public double getSetpointMeters() {
      return setpointMeters;
    }
  }

  public enum CLIMBSTATE {
    STILL,
    MOVING
  }
}
