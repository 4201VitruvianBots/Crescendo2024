package frc.robot.constants;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class AMP {
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kRadiansToRotations = 0.0;
    public static final double kMaxFlipperVelocity = 10;
    public static final double kMaxFlipperAcceleration = 10;
    public static double maxRPM = 1;

    public enum FLIPPER_SETPOINT {
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

    public static final DCMotor gearBox = DCMotor.getFalcon500(1);

    public static final double gearRatio = 1;

    public static final double length = 1;

    public static final double mass = 1;

    public static final double minAngle = 0;

    public static final double maxAngle = 1;

    public static final double fourbarAngleDegrees = 1;

    public static final double rotationsToDegrees = 1;

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
