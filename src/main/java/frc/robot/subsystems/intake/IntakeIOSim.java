package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  // I am completely useful :D
  /*
   *              ,--.    ,--.
                 ((O ))--((O ))
               ,'_`--'____`--'_`.
              _:  ____________  :_
             | | ||::::::::::|| | |
             | | ||::::::::::|| | |
             | | ||::::::::::|| | |
             |_| |/__________\| |_|
               |________________|
            __..-'            `-..__
         .-| : .----------------. : |-.
       ,\ || | |\______________/| | || /.
      /`.\:| | ||  __  __  __  || | |;/,'\
     :`-._\;.| || '--''--''--' || |,:/_.-':
     |    :  | || .----------. || |  :    |
     |    |  | || '----------' || |  |    |
     |    |  | ||   _   _   _  || |  |    |
     :,--.;  | ||  (_) (_) (_) || |  :,--.;
     (`-'|)  | ||______________|| |  (|`-')
      `--'   | |/______________\| |   `--'
             |____________________|
              `.________________,'
               (_______)(_______)
               (_______)(_______)
               (_______)(_______)
               (_______)(_______)
              |        ||        |
              '--------''--------'
   */
  // This is wonderful, thank you Mr. Garrett.

  // Real intake uses kraken 44, but it doesnt matter too much for sim.
  private static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60( 1);
  private static final DCMotor VACUUM_GEARBOX = DCMotor.getKrakenX60(1);
  private DCMotorSim intakeMotor;
  private DCMotorSim vacuumMotor;

  public IntakeIOSim(){
    intakeMotor =  new DCMotorSim(LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, IntakeConstants.INTAKE_JKG, IntakeConstants.INTAKE_GEARING), INTAKE_GEARBOX);
    vacuumMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(VACUUM_GEARBOX, IntakeConstants.VACUUM_JKG, IntakeConstants.VACUUM_GEARING), VACUUM_GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs){
    inputs.intakeAppliedVolts = intakeMotor.getInputVoltage();
    inputs.intakeCurrentAmps = new double[] { intakeMotor.getCurrentDrawAmps() };
    inputs.intakeVelocityRadPerSec = intakeMotor.getAngularVelocityRadPerSec();
    
    inputs.vacuumAppliedVolts = vacuumMotor.getInputVoltage();
    
    Logger.recordOutput("Intake/VelocityRadPerSec", intakeMotor.getAngularVelocityRadPerSec());
    Logger.recordOutput("Intake/AppliedVolts", intakeMotor.getInputVoltage());
    Logger.recordOutput("Intake/vacuumAppliedVolts", vacuumMotor.getInputVoltage());
  }

  @Override 
  public void setVoltage(double volts){
    intakeMotor.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void setVacuumVoltage(double volts){
    vacuumMotor.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }
}
