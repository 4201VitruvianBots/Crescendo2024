package frc.robot.visualizers;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.ARM;

public class ArmVisualizer implements AutoCloseable {
  private final Mechanism2d m_display = new Mechanism2d(1, 1);
  private MechanismRoot2d m_root2d;
  private MechanismLigament2d m_arm2d;
  private final MechanismLigament2d m_arm2dShared;
  private final Color8Bit m_ligamentColor = new Color8Bit(235, 137, 52);
  private final String m_name;

  public ArmVisualizer(String name) {
    m_name = name;

    // Create a "line" to represent the arm.
    // We will use this to show its current position
    m_arm2dShared = new MechanismLigament2d("m_" + m_name, ARM.length, 0);
  }

  public void displayVisualization() {
    m_arm2d = new MechanismLigament2d(m_name, ARM.length, 0);

    // Create a "point" to attach a MechanismLigament2d to.
    // In this case, we are using the center of the display
    m_root2d = m_display.getRoot(m_name, 0.5, 0.5);
    m_root2d.append(m_arm2d);

    SmartDashboard.putData(m_name, m_display);
  }

  public MechanismLigament2d getLigament() {
    return m_arm2dShared;
  }

  public void update(double angle, double velocity) {
    if (m_arm2d != null) {
      m_arm2d.setAngle(180 - angle);
      // Update the ligament color based on the module's current speed for easier visualization
      VisualizerUtils.updateMotorColor(m_arm2d, velocity, m_ligamentColor);
    }

    m_arm2dShared.setAngle(90 - angle);

    // Update the ligament color based on the module's current speed for easier visualization
    VisualizerUtils.updateMotorColor(m_arm2dShared, velocity, m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    if (m_root2d != null) {
      m_arm2d.close();
      m_root2d.close();
      m_display.close();
    }
    m_arm2dShared.close();
  }
}
