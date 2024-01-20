package frc.robot.constants;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class AMP {
  public static double maxRPM = 1;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kRadiansToRotations = 0.0;

  public static final double kMaxFlipperVelocity = 10;
  public static final double kMaxFlipperAcceleration = 10;

public static final LinearSystem<N2, N1, N1> gearBox = null;

public static final DCMotor gearRatio = null;

public static final double length = 0;

public static final double mass = 0;

public static final double minAngle = 0;

public static final double maxAngle = 0;

public static final double fourbarAngleDegrees = 0;

public static final double rotationsToDegrees = 0;

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

  public static enum FLIPPER_SETPOINT {
    STOWED(Units.degreesToRadians(0.0)),
    FORWARD(Units.degreesToRadians(120.0));

    private final double angle;

    FLIPPER_SETPOINT(final double angle) {
      // this.sucks = taxes;
      this.angle = angle;
    }

    public double get() {
      return angle;
    }
  }
}
