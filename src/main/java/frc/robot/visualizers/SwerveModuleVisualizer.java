package frc.robot.visualizers;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SwerveModuleVisualizer implements AutoCloseable {
  // Create a "canvas" to draw the module on
  private final Mechanism2d m_moduleVisualizer = new Mechanism2d(1, 1);
  private final MechanismRoot2d m_moduleRoot;
  private final MechanismLigament2d m_moduleLigament;
  private Color8Bit m_ligamentColor = new Color8Bit(Color.kGray);
  private String m_name;
  private double m_moduleMaxSpeedMps;

  private enum SPEED_COLOR {
    STOPPED(new Color8Bit(Color.kGray)),
    POSITIVE(new Color8Bit(Color.kGreen)),
    NEGATIVE(new Color8Bit(Color.kRed));

    private final Color8Bit value;

    SPEED_COLOR(final Color8Bit value) {
      this.value = value;
    }
  }

  public SwerveModuleVisualizer(String name, double moduleMaxSpeedMps) {
    m_name = name;
    m_moduleMaxSpeedMps = moduleMaxSpeedMps;

    // Create a "point" to attach a MechanismLigament2d to.
    // In this case, we are using the center of the swerve module.
    m_moduleRoot = m_moduleVisualizer.getRoot(m_name, 0.5, 0.5);

    // Create a "line" to represent the swerve module.
    // We will use this to show the current direction/speed of the swerve module.
    m_moduleLigament = m_moduleRoot.append(new MechanismLigament2d(m_name + "_direction", 0, 0));

    SmartDashboard.putData(m_name, m_moduleVisualizer);
  }

  public Mechanism2d getMechanism2d() {
    return m_moduleVisualizer;
  }

  public void update(SwerveModuleState state) {
    m_moduleLigament.setLength(state.speedMetersPerSecond / (2 * m_moduleMaxSpeedMps) + 0.25);
    m_moduleLigament.setAngle(state.angle.getDegrees());

    // Update the ligament color based on the module's current speed for easier visualization
    if (Math.abs(state.speedMetersPerSecond) < (m_moduleMaxSpeedMps * 0.01))
      m_ligamentColor = SPEED_COLOR.STOPPED.value;
    else if (state.speedMetersPerSecond > 0) {
      m_ligamentColor = SPEED_COLOR.POSITIVE.value;
    } else if (state.speedMetersPerSecond < 0) {
      m_ligamentColor = SPEED_COLOR.NEGATIVE.value;
    }

    m_moduleLigament.setColor(m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    m_moduleRoot.close();
    m_moduleLigament.close();
  }
}
