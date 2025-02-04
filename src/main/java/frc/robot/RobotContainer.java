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

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.arm.ArmConstants;
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

  private final VisionSubsystem vision;

  // private final ClimberSubsystem climber;
  private final ArmSubsystem arm;
  // private final IntakeSubsystem intake;

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
  @SuppressWarnings("unused")
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
            new VisionIOLimelight("llone"));

        arm = new ArmSubsystem(new ArmIOTalonFX());
        // climber = new ClimberSubsystem(new ClimberIOTalonFX());
        // intake = new IntakeSubsystem(new IntakeIOTalonFX());
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
            new VisionIOSim(
                new Transform3d(
                    Units.inchesToMeters(8.875),
                    0,
                    Units.inchesToMeters(8.25),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(-20.0),
                        0)),
                Rotation2d.fromDegrees(62.5),
                Rotation2d.fromDegrees(48.9)));

        arm = new ArmSubsystem(new ArmIOSim());
        // climber = new ClimberSubsystem(new ClimberIO() {
        // });
        // intake = new IntakeSubsystem(new IntakeIO() {});

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
        // intake = new IntakeSubsystem(new IntakeIO() {
        // });
        break;
    }

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

    NamedCommands.registerCommand("ArmState_Start", ArmCommands.updateSetpoint(arm, ArmStates.START));
    NamedCommands.registerCommand("ArmState_Intake", ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER));
    NamedCommands.registerCommand("ArmState_Low", ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
    NamedCommands.registerCommand("ArmState_Mid", ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF));
    NamedCommands.registerCommand("ArmState_High", ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L3_REEF));
    // NamedCommands.registerCommand("Set Arm Start State",
    // ArmCommands.updateSetpoint(arm, ArmStates.START));
    // NamedCommands.registerCommand("Set Arm Intake State",
    // ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF));
    // NamedCommands.registerCommand("Set Arm Feed State",
    // ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button -> command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
   * and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    switch (Constants.currentDriver) {
      case MATT:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -mattTranslation.StickYAxis(),
                () -> -mattTranslation.StickXAxis(),
                () -> mattRotation.StickXAxis()));

        mattRotation.B1().onTrue(Commands.runOnce(robotState::zeroHeading));

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
        programmingController.a().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.START));
        programmingController.b().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER));
        programmingController.povDown().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
        programmingController.povRight().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF));
        programmingController.povUp().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L3_REEF));

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
        programmingController.button(12).onTrue(Commands.runOnce(RobotState.getInstance()::zeroHeading));

        programmingController.button(1).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.START));

        programmingController.button(2).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER));

        programmingController.button(5).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));

        programmingController.button(4).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF));

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
    //     try{
    //     // Load the path you want to follow using its name in the GUI
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("Straight Path");

    //     // Create a path following command using AutoBuilder. This will also trigger event markers.
    //     return AutoBuilder.followPath(path);

    // } catch (Exception e) {
    //     DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //     return Commands.none();
    // }
  }
}
