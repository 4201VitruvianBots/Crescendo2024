package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class SHOOTER {
  public static double gearRatioBottom = 1.0;
  public static double gearRatioTop = 1.0;

  public static final int kSlotIdx = 0;

  public static final double kP = 10;
  public static final double kI = 0.0;
  public static final double kD = 1;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final double kShooterAngle = Units.degreesToRadians(35);

  public static final double kDistanceFromIntake = Units.inchesToMeters(19);

  public enum WAIT {
    WAIT_FOR_FLYWHEEL_SETPOINT(3),
    WAIT_FOR_AMP_SCORE(0.8);

    private final double value;

    WAIT(double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public enum RPM_SETPOINT {
    NONE(0),
    COOLVALUE(4201),
    MAX(6000);

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

    public static final double BlueSpeakerTLY = 6.1478414;
    public static final double BlueSpeakerTLX = 0.464947;

    public static final double BlueSpeakerTRY = 5.0232564;
    public static final double BlueSpeakerTRX = 0.464947;

    public static final double BlueSpeakerBLY = 6.1478414;
    public static final double BlueSpeakerBLX = 0;

    public static final double BlueSpeakerBRY = 5.0232564;
    public static final double BlueSpeakerBRX = 0;

    public static final double RedSpeakerTLY = 6.1478414;
    public static final double RedSpeakerTLX = 16.0697672;

    public static final double RedSpeakerTRY = 5.0232564;
    public static final double RedSpeakerTRX = 16.0697672;

    public static final double RedSpeakerBLY = 6.1478414;
    public static final double RedSpeakerBLX = 16.5410642;

    public static final double RedSpeakerBRY = 5.0232564;
    public static final double RedSpeakerBRX = 16.5410642;
  }
}
