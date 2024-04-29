package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class SHOOTER {
  public static final double kPBottom = 7;
  public static final double kIBottom = 0;
  public static final double kDBottom = 0;

  public static final double kPTop = 7;
  public static final double kITop = 0;
  public static final double kDTop = 0;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final double kShooterAngle = Units.degreesToRadians(35);

  public static final double kBottomFlywheelDistanceFromIntake = Units.inchesToMeters(25.9);
  public static final double kBottomFlywheelDistanceFromDriveBase = Units.inchesToMeters(6.6);
  public static final double kTopFlywheelDistanceFromIntake = Units.inchesToMeters(22.4);
  public static final double kTopFlywheelDistanceFromDriveBase = Units.inchesToMeters(11.5);

  public static final double gearRatioBottom = 20.0 / 28.0;
  public static final DCMotor ShooterBottomGearbox = DCMotor.getKrakenX60(1);

  public static final DCMotor ShooterTopGearbox = DCMotor.getKrakenX60(1);
  public static final double gearRatioTop = 20.0 / 28.0;

  public static final double Inertia = 0.001;

  public static final double flywheelSize = Units.inchesToMeters((3.4 * Math.PI) / 8);

  public enum WAIT {
    WAIT_FOR_FLYWHEEL_SETPOINT(3),
    WAIT_FOR_AMP_SCORE(0.8),
    SHOOTING(0.75);

    private final double value;

    WAIT(double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public enum RPM_SETPOINT {
    REVERSE(-0.1),
    TOLERANCE(450),
    // TOLERANCE_AUTO(600),

    NONE(0),

    SLOW(3000),

    SLOW2(1750),

    SLOW3(4500),

    MIN_RAB(1000),
    MAX_RAB(3000),

    SPEAKERBOTTOM(8000),
    SPEAKERTOP(6000),

    SPEAKER(7000),
    AUTO_RPM(7600),
    MAX(8200); // This is actually 8000

    private final double value;

    RPM_SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public static final double NoteVelocity = 0; // add note velocity
}
