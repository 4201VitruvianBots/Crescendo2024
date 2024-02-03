package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Velocity;

public final class ARM {
  /* Static Feedforward Gain
     This is added to the closed loop output. The sign is determined by target velocity.
     The unit for this constant is dependent on the control mode,
     typically fractional duty cycle, voltage, or torque current */
  public static final double kS = 0.06;
  
  /* Velocity Feedforward Gain
     The units for this gain is dependent on the control mode.
     Since this gain is multiplied by the requested velocity,
     the units should be defined as units of output per unit of requested input velocity.
     For example, when controlling velocity using a duty cycle closed loop,
     the units for the velocity feedfoward gain will be duty cycle per requested rps, or 1/rps. */
  public static final double kV = 1.6;
  
  /* A higher P value means you will put more effort into correcting the measured error,
     but it means you can overshoot your target and then the response will look like an oscillating graph. */
  public static final double kP = 0.085;
  
  /* I value is generally used to correct steady-state error
     (e.g. your goal is 100 but you are at 99, so the sum of error
     over time will let it correct for that final error). */
  public static final double kI = 0.0;
  
  /* D is generally used to 'predict' the next output using the slope of the error,
     so it is usually used with P to get a fast, but accurate response. */
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
      return angle * 4; // Temporary fix, TODO: remove
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
