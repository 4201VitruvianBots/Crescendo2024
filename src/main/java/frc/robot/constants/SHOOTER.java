package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class SHOOTER {

  public static final int kSlotIdx = 0;

  public static final double topkP = 6;
  public static final double topkI = 0;
  public static final double topkD = 0;

  public static final double bottomkP = 4;
  public static final double bottomkI = 0;
  public static final double bottomkD = 0;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final double kShooterAngle = Units.degreesToRadians(35);

  public static final double kTopFlywheelDistanceFromIntake = Units.inchesToMeters(22.4);
  public static final double kTopFlywheelDistanceFromDriveBase = Units.inchesToMeters(11.5);
  public static final double kBottomFlywheelDistanceFromIntake = Units.inchesToMeters(25.9);
  public static final double kBottomFlywheelDistanceFromDriveBase = Units.inchesToMeters(6.6);

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
    TOLERANCE(200),

    NONE(0),

    SLOW(600),
    SPEAKER(7000),
    MAX(8000);

    private final double value;

    RPM_SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public static final double NoteVelocity = 0; // add note velocity

  public static class SPEAKER {


    public static final double SpeakerTopLeftY = 6.1478414;
    public static double SpeakerTopLeftX = 0.464947;

    public static final double SpeakerTopRightY = 5.0232564;
    public static double SpeakerTopRightX = 0.464947;

    public static final double SpeakerBottomLeftY = 6.1478414;
    public static double SpeakerBottomLeftX = 0;

    public static final double SpeakerBottomRightY = 5.0232564;
    public static double SpeakerBottomRightX = 0;


    public static void initConstants(boolean isBlueAlliance) {
      if(isBlueAlliance)
      {
        SpeakerTopLeftX = 16.0697672;
        SpeakerTopRightX = 16.0697672;
        SpeakerBottomLeftX = 16.5410642;
        SpeakerBottomRightX = 16.5410642;
      }

    }

  }
}
