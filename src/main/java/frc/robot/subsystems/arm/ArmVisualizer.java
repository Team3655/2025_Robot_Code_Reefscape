package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer {

  public final LoggedMechanism2d arm;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d shoulder;
  private final LoggedMechanismLigament2d elbow;
  private final LoggedMechanismLigament2d wrist;
  private final LoggedMechanismLigament2d base;
      
  public ArmVisualizer(String key) {

    // Creates the setup for the mechanism2D
    arm = new LoggedMechanism2d(3, 3);
    // Sets starting position for the arm
    root = arm.getRoot("arm", 1.5, 0);

    switch (key) {
      case "Current":
        // Appends the arm ligaments onto other ligaments with colors to visualize real
        // robot or simulated data.
        base = root.append(new LoggedMechanismLigament2d(
            "Base",
            ArmConstants.H_TOWER_GROUND_HEIGHT_METERS,
            90,
            7,
            new Color8Bit(Color.kGray)));
        shoulder = base.append(new LoggedMechanismLigament2d(
            "Shoulder",
            ArmConstants.SHOULDER_LENGTH_METERS,
            -65,
            6,
            new Color8Bit(Color.kRed)));
        elbow = shoulder.append(new LoggedMechanismLigament2d(
            "Elbow",
            ArmConstants.ELBOW_LENGTH_METERS,
            90,
            4,
            new Color8Bit(Color.kBlue)));
        wrist = elbow.append(new LoggedMechanismLigament2d(
          "Wrist", 
          ArmConstants.WRIST_LENGTH_METERS, 
          0, 
          2,
            new Color8Bit(Color.kGreen)));
        break;

      case "Setpoint":

        // Appends the arm ligaments onto other ligaments without color to visualize the
        // setpoint.
        // Should be the same in sim and real modes.
        base = root.append(new LoggedMechanismLigament2d(
          "Base",
          ArmConstants.H_TOWER_GROUND_HEIGHT_METERS,
          90,
          7,
          new Color8Bit(Color.kYellow)));
      shoulder = base.append(new LoggedMechanismLigament2d(
          "Shoulder",
          ArmConstants.SHOULDER_LENGTH_METERS,
          -65,
          6,
          new Color8Bit(Color.kYellow)));
      elbow = shoulder.append(new LoggedMechanismLigament2d(
          "Elbow",
          ArmConstants.ELBOW_LENGTH_METERS,
          90,
          4,
          new Color8Bit(Color.kYellow)));
      wrist = elbow.append(new LoggedMechanismLigament2d(
        "Wrist", 
        ArmConstants.WRIST_LENGTH_METERS, 
        0, 
        2,
          new Color8Bit(Color.kYellow)));
        break;

      default:
        base = null;
        shoulder = null;
        elbow = null;
        wrist = null;
        break;
    }

    shoulder.setAngle(-65);
    elbow.setAngle(105);

  }

  /**
   * Updates the visualizer to the given values
   * 
   * @param shoulderRelative Shoulder angle value
   * @param elbowRelative Elbow angle value
   * @param wristRelative Wrist angle value
   */
  public void update(double shoulderRelative, double elbowRelative, double wristRelative) {
    shoulder.setAngle(shoulderRelative);
    elbow.setAngle(elbowRelative);
    wrist.setAngle(wristRelative);
  }
}
