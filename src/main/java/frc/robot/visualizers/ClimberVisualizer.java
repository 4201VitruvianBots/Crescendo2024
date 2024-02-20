package frc.robot.visualizers;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.CLIMBER;

public class ClimberVisualizer implements AutoCloseable {
  private MechanismLigament2d m_post2d;
  private MechanismLigament2d m_climber2d;
  private MechanismLigament2d m_hook2d;
  private final Color8Bit m_ligamentColor = new Color8Bit(52, 212, 235);
  private final String m_name;

  public ClimberVisualizer(String name) {
    m_name = name;

    // Create lines to represent the climber post
    m_post2d = new MechanismLigament2d(m_name + "Post", CLIMBER.kPostHeight, 90);

    // Create lines to represent the climber hook
    // We will use this to show its current position
    m_climber2d = new MechanismLigament2d(m_name, CLIMBER.kHookHeight, 90);
    m_hook2d = new MechanismLigament2d(m_name + "Hook", CLIMBER.kHookLength, 90);
    m_climber2d.append(m_hook2d);
  }

  public MechanismLigament2d getLigament() {
    return m_climber2d;
  }

  public MechanismLigament2d getPost() {
    return m_post2d;
  }

  public void update(double height, double velocity) {
    m_climber2d.setLength(height + CLIMBER.kHookHeight);

    // Update the ligament color based on the module's current speed for easier visualization
    VisualizerUtils.updateMotorColor(m_post2d, velocity, m_ligamentColor);
    VisualizerUtils.updateMotorColor(m_climber2d, velocity, m_ligamentColor);
    VisualizerUtils.updateMotorColor(m_hook2d, velocity, m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    m_post2d.close();
    m_climber2d.close();
    m_hook2d.close();
  }
}
