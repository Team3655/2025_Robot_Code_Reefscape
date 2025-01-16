package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ArmIOSim implements ArmIO {

    private final PIDController shoulderController = new PIDController(0, 0, 0);
    private final PIDController elbowController = new PIDController(0, 0, 0);
    private final PIDController wrisController = new PIDController(0, 0, 0);
    private final Encoder encoder = new Encoder(0, 1);

    private final DCMotor gearbox = DCMotor.getKrakenX60(3);

    private final SingleJointedArmSim shoulderSim = new SingleJointedArmSim(gearbox, ArmConstants.SHOULDER_REDUCTION,
            SingleJointedArmSim.estimateMOI(ArmConstants.SHOULDER_LENGTH_METERS, ArmConstants.SHOULDER_MASS_KG),
            ArmConstants.SHOULDER_LENGTH_METERS, ArmConstants.SHOULDER_MIN_ANGLE_RADS.getRadians(),
            ArmConstants.SHOULDER_MAX_ANGLE_RADS.getRadians(), true, 0);

    private final SingleJointedArmSim elbowSim = new SingleJointedArmSim(gearbox, ArmConstants.ELBOW_REDUCTION,
            SingleJointedArmSim.estimateMOI(ArmConstants.ELBOW_LENGTH_METERS, ArmConstants.ELBOW_MASS_KG),
            ArmConstants.ELBOW_LENGTH_METERS, ArmConstants.ELBOW_MIN_ANGLE_RADS.getRadians(),
            ArmConstants.ELBOW_MAX_ANGLE_RADS.getRadians(), true, Rotation2d.fromDegrees(115).getRadians());

    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(gearbox, ArmConstants.WRIST_REDUCTION,
            SingleJointedArmSim.estimateMOI(ArmConstants.WRIST_LENGTH_METERS, ArmConstants.WRIST_MASS_KG),
            ArmConstants.WRIST_LENGTH_METERS, ArmConstants.WRIST_MIN_ANGLE_RADS.getRadians(),
            ArmConstants.WRIST_MAX_ANGLE_RADS.getRadians(), true, Rotation2d.fromDegrees(115).getRadians());

    private final Mechanism2d arm = new Mechanism2d(100, 100);
    private final MechanismRoot2d root = arm.getRoot("arm", 2, 0);
    

}
