package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class ARM {
  public static final double kS = 0.06;
  public static final double kV = 1.6;
  public static final double kP = 0.085;
  public static final double kI = 0.0;
  public static final double kD = 13.0;

  public static final double kMaxArmVelocity = 10;
  public static final double kMaxArmAcceleration = 10;

  public enum ARM_SETPOINT {
    STOWED(Units.degreesToRotations(0.0)),
    FORWARD(Units.degreesToRotations(120.0));

    private final double angle;

    ARM_SETPOINT(final double angle) {
      // this.sucks = taxes;
      this.angle = angle;
    }

    public double get() {
      return angle;
    }
  }

  public static final DCMotor gearBox = DCMotor.getFalcon500(1);

  // Jacob said the gear ratio is 1:140 but WPILIB doesn't seem to like that
  public static final double gearRatio = 140.0 / 1.0;

  public static final double length = Units.inchesToMeters(21.5);

  // TOOD: Find actual mass of arm
  public static final double mass = Units.lbsToKilograms(20.0);

  public static final double minAngleDegrees = -70;

  public static final double maxAngleDegrees = 160;

  public static final double startingAngleDegrees = minAngleDegrees;

  public static final double mountingAngleDegrees = 0;

  public enum AMP_STATE {
    NONE(0),
    SCORE(0.8);

    private final double value;

    AMP_STATE(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
