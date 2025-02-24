// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CommandNXT;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final RobotState robotState = RobotState.getInstance();

  // Subsystems
  private final DriveSubsystem drive;

  @SuppressWarnings("unused")
  private final VisionSubsystem vision;

  // private final ClimberSubsystem climber;
  private final ArmSubsystem arm;
  private final IntakeSubsystem intake;

  // Controller
  // programming controller
  private final CommandXboxController programmingController = new CommandXboxController(5);

  // Matt's controller
  private final CommandNXT mattTranslation = new CommandNXT(0);
  private final CommandNXT mattRotation = new CommandNXT(1);

  // Ethan's controller
  private final CommandNXT ethanTranslation = new CommandNXT(2);
  private final CommandNXT ethanRotation = new CommandNXT(3);

  // Operator controller
  private final CommandGenericHID tractorController = new CommandGenericHID(4);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new DriveSubsystem(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));

        vision = new VisionSubsystem(
            new VisionIOLimelight("limelight-back"),
            new VisionIOLimelight("limelight-left"),
            new VisionIOLimelight("limelight-right"));

        arm = new ArmSubsystem(new ArmIOTalonFX());
        // climber = new ClimberSubsystem(new ClimberIOTalonFX());
        intake = new IntakeSubsystem(new IntakeIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new DriveSubsystem(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

        vision = new VisionSubsystem(
            new VisionIOSim("left", VisionConstants.LEFT_ROBOT_TO_CAMERA),
            new VisionIOSim("right", VisionConstants.RIGHT_ROBOT_TO_CAMERA));

        arm = new ArmSubsystem(new ArmIOSim());
        // climber = new ClimberSubsystem(new ClimberIO() {
        // });
        intake = new IntakeSubsystem(new IntakeIOSim() {
        });

        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new DriveSubsystem(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        vision = new VisionSubsystem(
            new VisionIO() {
            });

        arm = new ArmSubsystem(new ArmIO() {
        });

        // climber = new ClimberSubsystem(new ClimberIO() {
        // });
        intake = new IntakeSubsystem(new IntakeIO() {
        });
        break;
    }

    NamedCommands.registerCommand("ArmState_Start", ArmCommands.updateSetpoint(arm, ArmStates.START));
    NamedCommands.registerCommand("ArmState_Intake", Commands.parallel(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER), IntakeCommands.runIntake(intake, -6)));
    NamedCommands.registerCommand("ArmState_L1", ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
    NamedCommands.registerCommand("ArmState_L2", ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF));
    NamedCommands.registerCommand("ArmState_L3", ArmCommands.updateSetpoint(arm, ArmStates.REAR_L3_REEF));
    NamedCommands.registerCommand("ArmState_L4", ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF));
    NamedCommands.registerCommand("Place", IntakeCommands.runIntake(intake, 6));
    NamedCommands.registerCommand("Stop_Intake", IntakeCommands.stopIntake(intake));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button -> command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}) and then
   * passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    switch (Constants.currentDriver) {
            case MATT:
              drive.setDefaultCommand(
                  DriveCommands.joystickDrive(
                      drive,
                      () -> mattTranslation.StickYAxis(),
                      () -> mattTranslation.StickXAxis(),
                      () -> -mattRotation.StickXAxis()));
      
              mattTranslation.B1().onTrue(Commands.runOnce(robotState::zeroHeading));
      
              tractorController.button(9).onTrue(IntakeCommands.runIntake(intake, 6))
                  .onFalse(IntakeCommands.stopIntake(intake));
      
              tractorController.button(10).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.START).alongWith(IntakeCommands.stopIntake(intake)));
              tractorController.button(5)
                  .onTrue(Commands
                      .sequence(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER),IntakeCommands.runIntake(intake, -6)))
                  .onFalse(Commands
                      .sequence(IntakeCommands.stopIntake(intake), ArmCommands.updateSetpoint(arm, ArmStates.START)));
              tractorController.button(6).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
              tractorController.button(1)
                  .onTrue(Commands
                      .parallel(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF), IntakeCommands.runIntake(intake, -2)));
              // tractorController.button(5).onTrue(ArmCommands.updateSetpoint(arm,
              // ArmStates.FRONT_L3_REEF));
              tractorController.button(2)
                  .onTrue(Commands
                      .parallel(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L3_REEF), IntakeCommands.runIntake(intake, -2)));
              tractorController.button(3)
                  .onTrue(Commands
                      .parallel(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF), IntakeCommands.runIntake(intake,-2)));

              // Close valve and create vacuum pressure
              tractorController.button(11)
                  .onTrue(Commands
                      .parallel(IntakeCommands.toggleVacuum(intake, false, 12), 
                                    ArmCommands.updateSetpoint(arm, ArmStates.Algie_Store)));
              // Close valve and create vacuum pressure
              tractorController.button(13)
                  .onTrue(Commands
                      .parallel(IntakeCommands.toggleVacuum(intake, false, 12), 
                                ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_Algie)));
              // Open valve and create positive pressure
              // Turn off vacuum when released
              tractorController.button(12)
                  .onTrue(Commands
                    .parallel(IntakeCommands.toggleVacuum(intake, true, -12), 
                                ArmCommands.updateSetpoint(arm, ArmStates.START)))
                  .onFalse(IntakeCommands.toggleVacuum(intake, false, 0));
      
              break;
            case ETHAN:
              drive.setDefaultCommand(
                  DriveCommands.joystickDrive(
                      drive,
                      () -> -ethanTranslation.StickYAxis(),
                      () -> -ethanTranslation.StickXAxis(),
                      () -> -ethanRotation.StickXAxis()));
      
              break;
            case PROGRAMMING:
              drive.setDefaultCommand(
                  DriveCommands.joystickDrive(
                      drive,
                      () -> -programmingController.getLeftY(),
                      () -> -programmingController.getLeftX(),
                      () -> -programmingController.getRightX()));
      
              // programmingController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
              programmingController.button(8).onTrue(Commands.runOnce(robotState::zeroHeading));
      
              // programmingController.a().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.START));
              // programmingController.b().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER));
              // programmingController.povDown().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
              // programmingController.povRight().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF));
              // programmingController.povUp().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L3_REEF));
              // programmingController.povLeft().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF));
      
              programmingController.rightBumper().whileTrue(IntakeCommands.runIntake(intake, -6))
                  .onFalse(IntakeCommands.stopIntake(intake));
      
              programmingController.leftBumper().whileTrue(IntakeCommands.runIntake(intake, 6))
                  .onFalse(IntakeCommands.stopIntake(intake));

              // Close valve and create vacuum pressure
              programmingController.a()
                  .onTrue(IntakeCommands.toggleVacuum(intake, false, 12));

              // Open valve and create positive pressure
              // Turn off vacuum when released
              programmingController.b()
                  .onTrue(IntakeCommands.toggleVacuum(intake, true, -12))
                  .onFalse(IntakeCommands.toggleVacuum(intake, false, 0));
            



        // programmingController.rightBumper().whileTrue(arm.sysIdDynamic(Direction.kForward));
        // programmingController.leftBumper().whileTrue(arm.sysIdQuasistatic(Direction.kForward));
        break;

      case MACBOOK:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> programmingController.getRawAxis(1),
                () -> programmingController.getRawAxis(0),
                () -> -programmingController.getRawAxis(2)));

        programmingController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        programmingController.button(12).onTrue(Commands.runOnce(robotState::zeroHeading));

        programmingController.button(1).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.START));
        programmingController.button(2).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER));
        programmingController.povDown().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
        programmingController.povRight().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF));
        programmingController.povUp().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L3_REEF));
        programmingController.povLeft().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF));

        // programmingController.rightBumper().onTrue(IntakeCommands.runIntake(intake,
        // 12)).onFalse(IntakeCommands.stopIntake(intake));
        // programmingController.leftBumper().onTrue(IntakeCommands.runIntake(intake,
        // -12)).onFalse(IntakeCommands.stopIntake(intake));
        // programmingController.povUp().onTrue(IntakeCommands.runVacuum(intake,
        // 12)).onFalse(IntakeCommands.stopVacuum(intake));
        // programmingController.y().onTrue(ClimbCommands.runClimber(climber,
        // 5)).onFalse(ClimbCommands.stopClimber(climber));
        // programmingController.povUp().onTrue(ClimbCommands.setArmClimbPosition(climber));
        // programmingController.povDown().onTrue(ClimbCommands.setArmInitPosition(climber));

        // programmingController.a().onTrue(Commands.runOnce(() ->
        // arm.updateSetpoint(ArmStates.FRONT_FEEDER), arm));
        // programmingController.b().onTrue(Commands.runOnce(() ->
        // arm.updateSetpoint(ArmStates.START), arm));
        break;
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
