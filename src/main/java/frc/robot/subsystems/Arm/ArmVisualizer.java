package frc.robot.subsystems.Arm;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Arm.ArmSubsystem.ArmPose;

public class ArmVisualizer {

    private final LoggedMechanism2d arm;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d shoulder;
    private final LoggedMechanismLigament2d elbow;
    private final LoggedMechanismLigament2d wrist;
    private final LoggedMechanismLigament2d base;
    private final String key;

    public ArmVisualizer(String key){
        this.key = key;

        arm = new LoggedMechanism2d(3, 3);
        root = arm.getRoot("arm", 1.5, 0);

        switch (key) {
            case "current":
            base = root.append(new LoggedMechanismLigament2d("Base", 1, 90, 7, new Color8Bit(Color.kGray)));
            shoulder = base.append(new LoggedMechanismLigament2d("Shoulder", ArmConstants.SHOULDER_LENGTH_METERS, 200, 6, new Color8Bit(Color.kRed)));
            elbow = shoulder.append(new LoggedMechanismLigament2d("Elbow", ArmConstants.ELBOW_LENGTH_METERS, 90, 4, new Color8Bit(Color.kBlue)));
            wrist = elbow.append(new LoggedMechanismLigament2d("Wrist", ArmConstants.WRIST_LENGTH_METERS, 90, 2 , new Color8Bit(Color.kGreen)));
                break;

            case "setpoint":
            base = root.append(new LoggedMechanismLigament2d("Base", 1, 90, 7, new Color8Bit(Color.kGray)));
            shoulder = base.append(new LoggedMechanismLigament2d("Shoulder", ArmConstants.SHOULDER_LENGTH_METERS, 200, 6, new Color8Bit(Color.kYellow)));
            elbow = shoulder.append(new LoggedMechanismLigament2d("Elbow", ArmConstants.ELBOW_LENGTH_METERS, 90, 4, new Color8Bit(Color.kYellow)));
            wrist = elbow.append(new LoggedMechanismLigament2d("Wrist", ArmConstants.WRIST_LENGTH_METERS, 90, 2 , new Color8Bit(Color.kYellow)));
                break;
            default:
            base = null;
            shoulder = null;
            elbow  = null;
            wrist = null;
                break;
        }
    }

    public void update(ArmPose pose){
        shoulder.setAngle(pose.shoulderAngle());
        elbow.setAngle(pose.elbowAngle());
        wrist.setAngle(pose.wristAngle());   
        
        Logger.recordOutput("Arm/Mechanism2d" + key, arm);
    }
}
