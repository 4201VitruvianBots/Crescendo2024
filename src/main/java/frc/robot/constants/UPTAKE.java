package frc.robot.constants;

public final class UPTAKE {


  public enum UPTAKE_STATE {
    DOWNTAKE(-0.8),
    NONE(0),
    UPTAKE(0.8);
    
    private final double value;

      UPTAKE_STATE(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
    }
  }
}