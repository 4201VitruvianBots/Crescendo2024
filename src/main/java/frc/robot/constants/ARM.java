package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class ARM {
  /* Static Feedforward Gain
  This is added to the closed loop output. The sign is determined by target velocity.
  The unit for this constant is dependent on the control mode,
  typically fractional duty cycle, voltage, or torque current */
  public static final double kS = 0;

  /* Velocity Feedforward Gain
  The units for this gain is dependent on the control mode.
  Since this gain is multiplied by the requested velocity,
  the units should be defined as units of output per unit of requested input velocity.
  For example, when controlling velocity using a duty cycle closed loop,
  the units for the velocity feedfoward gain will be duty cycle per requested rps, or 1/rps. */
  public static final double kV = 0;

  /* A higher P value means you will put more effort into correcting the measured error,
  but it means you can overshoot your target and then the response will look like an oscillating graph. */
  public static final double kP = 75.0;

  /* I value is generally used to correct steady-state error
  (e.g. your goal is 100 but you are at 99, so the sum of error
  over time will let it correct for that final error). */
  public static final double kI = 0.0;

  /* D is generally used to 'predict' the next output using the slope of the error,
  so it is usually used with P to get a fast, but accurate response. */
  public static final double kD = 0.0;

  public static final double kMaxArmVelocity = 10;
  public static final double kMaxArmAcceleration = 10;

  public enum ARM_SETPOINT {
    STOWED(Units.degreesToRotations(-40.0)),
    STAGED(Units.degreesToRotations(75.0)),
    FORWARD(Units.degreesToRotations(140.0)),
    TRAP(Units.degreesToRotations(130.0));

    private final double angle;

    ARM_SETPOINT(final double angle) {
      // this.sucks = taxes;
      this.angle = angle;
    }

    public double get() {
      return angle;
    }
  }

  public static final DCMotor gearBox = DCMotor.getKrakenX60(1);

  // Jacob said the gear ratio is 1:140 but WPILIB doesn't seem to like that
  public static final double gearRatio = 140.0 / 1.0;

  public static final double jointLength = Units.inchesToMeters(5);
  public static final double armVisualizerLength = Units.inchesToMeters(17);
  public static final double armLength = Units.inchesToMeters(21.5);

  public static final double mass = Units.lbsToKilograms(7.0);

  public static final double minAngleDegrees = -40;

  public static final double maxAngleDegrees = 140;

  public static final double startingAngleDegrees = minAngleDegrees;

  public static final double mountingAngleDegrees = 0;

  public static final double maxOutput = 0.4;
  public static final double joystickMultiplier = maxOutput;
}
