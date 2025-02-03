package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {

  public static final CANBus BUS = new CANBus("rio");
  public static final int VAC_MOTOR_PORT = 40;
  public static final int INTAKE_MOTOR_PORT = 41;
  public static final int CANRANGE_PORT = 42;

  public static final double INTAKE_JKG = 0.025;
  public static final double INTAKE_GEARING = 1;
  public static final double VACUUM_JKG = 0.025;
  public static final double VACUUM_GEARING = 1;

  public static final double CANRANGE_DETECTION_RANGE = Units.inchesToMeters(3);
}
