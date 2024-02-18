package frc.robot.visualizers;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.CLIMBER;

public class ClimberVisualizer implements AutoCloseable {
  private final Mechanism2d m_display = new Mechanism2d(1, 1);
  private MechanismRoot2d m_root2d;
  private MechanismRoot2d m_postRoot2d;
  private MechanismLigament2d m_post2d;
  private MechanismLigament2d m_climber2d;
  private MechanismLigament2d m_hook2d;
  private final MechanismLigament2d m_post2dShared;
  private final MechanismLigament2d m_climber2dShared;
  private final MechanismLigament2d m_hook2dShared;
  private Color8Bit m_ligamentColor = new Color8Bit(52, 212, 235);
  private final String m_name;

  public ClimberVisualizer(String name) {
    m_name = name;


    // Create lines to represent the climber post
    m_post2dShared = new MechanismLigament2d("m_" +  m_name + "Post", CLIMBER.kPostHeight, 90);

    // Create lines to represent the climber hook
    // We will use this to show its current position
    m_climber2dShared = new MechanismLigament2d("m_" +  m_name, CLIMBER.kHookHeight, 90);
    m_hook2dShared = new MechanismLigament2d("m_" +  m_name + "Hook", CLIMBER.kHookLength, 90);
    m_climber2dShared.append(m_hook2dShared);
  }

  public void displayVisualization() {
    // To make a separate individual display, we need to basically copy the drawing,
    // otherwise, the part will only show on one Mechanism2d
    m_post2d = new MechanismLigament2d(m_name + "Post", CLIMBER.kPostHeight, 90);
    m_climber2d = new MechanismLigament2d(m_name, CLIMBER.kHookHeight, 90);
    m_hook2d = new MechanismLigament2d(m_name + "Hook", CLIMBER.kHookLength, 90);
    m_climber2d.append(m_hook2d);

    // Create a "point" to attach a MechanismLigament2d to.
    // In this case, we are using a point offset of the center
    m_root2d = m_display.getRoot(m_name, 0.6, 0.25);
    m_root2d.append(m_climber2d);

    m_postRoot2d = m_display.getRoot(m_name, 0.7, 0.25);
    m_postRoot2d.append(m_post2d);

    SmartDashboard.putData(m_name, m_display);
  }

  public MechanismLigament2d getLigament() {
    return m_climber2dShared;
  }

  public MechanismLigament2d getPost() {
    return m_post2dShared;
  }

  public void update(double height, double velocity) {
    if (m_climber2d != null) {
      m_climber2d.setLength(height + CLIMBER.kHookHeight);

      // Update the ligament color based on the module's current speed for easier visualization
      VisualizerUtils.updateMotorColor(m_post2d, velocity, m_ligamentColor);
      VisualizerUtils.updateMotorColor(m_climber2d, velocity, m_ligamentColor);
      VisualizerUtils.updateMotorColor(m_hook2d, velocity, m_ligamentColor);
    }

    m_climber2dShared.setLength(height + CLIMBER.kHookHeight);

    // Update the ligament color based on the module's current speed for easier visualization
    VisualizerUtils.updateMotorColor(m_post2dShared, velocity, m_ligamentColor);
    VisualizerUtils.updateMotorColor(m_climber2dShared, velocity, m_ligamentColor);
    VisualizerUtils.updateMotorColor(m_hook2dShared, velocity, m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    if(m_root2d != null) {
      m_post2d.close();
      m_climber2d.close();
      m_hook2d.close();
      m_root2d.close();
      m_postRoot2d.close();
      m_display.close();
    }
    m_post2dShared.close();
    m_climber2dShared.close();
    m_hook2dShared.close();
  }
}
