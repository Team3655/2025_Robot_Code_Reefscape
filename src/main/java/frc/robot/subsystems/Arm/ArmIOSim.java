package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {

        private final DCMotor gearbox = DCMotor.getKrakenX60(1);

        private double shoulderVolts = 0.0;
        private double elbowVolts = 0.0;
        private double wristVolts = 0.0;

        private final SingleJointedArmSim shoulderSim = new SingleJointedArmSim(gearbox,
                        ArmConstants.SHOULDER_REDUCTION,
                        SingleJointedArmSim.estimateMOI(ArmConstants.SHOULDER_LENGTH_METERS,
                                        ArmConstants.SHOULDER_MASS_KG),
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

        @Override
        public void updateInputs(ArmIOInputs inputs) {

                shoulderSim.setInputVoltage(shoulderVolts);
                elbowSim.setInputVoltage(elbowVolts);
                wristSim.setInputVoltage(wristVolts);

                shoulderSim.update(0.02);
                elbowSim.update(0.02);
                wristSim.update(0.02);

                inputs.shoulderPosition = Rotation2d.fromDegrees(shoulderSim.getAngleRads());
                inputs.shoulderCurrentAmps = new double[] { shoulderSim.getCurrentDrawAmps() };
                inputs.shoulderVelocityRadPerSec = shoulderSim.getVelocityRadPerSec();

                inputs.elbowPosition = Rotation2d.fromDegrees(elbowSim.getAngleRads());
                inputs.elbowCurrentAmps = new double[] { elbowSim.getCurrentDrawAmps() };
                inputs.elbowVelocityRadPerSec = elbowSim.getVelocityRadPerSec();

                inputs.wristPosition = Rotation2d.fromDegrees(wristSim.getAngleRads());
                inputs.wristCurrentAmps = new double[] { wristSim.getCurrentDrawAmps() };
                inputs.wristVelocityRadPerSec = wristSim.getVelocityRadPerSec();
        }

        @Override
        public void setShoulderVoltage(double volts) {
                shoulderVolts = MathUtil.clamp(volts, -12, 12);
        }

        @Override
        public void setElbowVoltage(double volts) {
                elbowVolts = MathUtil.clamp(volts, -12, 12);
        }

        @Override
        public void setWristVoltage(double volts) {
                wristVolts = MathUtil.clamp(volts, -12, 12);
        }

}
