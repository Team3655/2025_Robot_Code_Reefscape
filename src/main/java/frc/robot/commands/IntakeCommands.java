package frc.robot.commands;

import java.lang.Thread.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommands {
    public static Command runIntake(IntakeSubsystem intake, double volts) {
        return Commands.runOnce(() -> intake.setIntakeVoltage(volts), intake);
    }

    public static Command runVacuum(IntakeSubsystem intake, double volts) {
        return Commands.runOnce(() -> intake.setVacuumVoltage(volts), intake);
    }

    public static Command stopIntake(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.setIntakeVoltage(0), intake);
    }

    public static Command stopVacuum(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.setVacuumVoltage(0), intake);
    }

    public static Command toggleVacuumSolenoid(boolean state, IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.toggleVacuumSolenoid(state));
    }

    public static Command toggleVacuum(IntakeSubsystem intake, boolean solenoidState, double volts) {
      return Commands.runOnce(() -> intake.toggleVacuum(solenoidState, volts), intake);
    }
}
