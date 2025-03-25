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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CommandNXT;
import frc.robot.util.FieldUtil;

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

    private final ClimberSubsystem climber;
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    // Controller
    // Programming controller
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
    private double driveMultiplier = 0.4;
    private double garrettDriveMultiplier = 0.2;

    private final FieldUtil fieldUtil = new FieldUtil();

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
                        new VisionIOLimelight("limelight-left"),
                        new VisionIOLimelight("limelight-right"),                       
                        new VisionIOLimelight("limelight-back"));

                arm = new ArmSubsystem(new ArmIOTalonFX());
                // climber = new ClimberSubsystem(new ClimberIOTalonFX());
                intake = new IntakeSubsystem(new IntakeIOReal());

                climber = new ClimberSubsystem(new ClimberIOTalonFX());
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

                climber = new ClimberSubsystem(new ClimberIO() {
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

                climber = new ClimberSubsystem(new ClimberIO() {
                });
                intake = new IntakeSubsystem(new IntakeIO() {
                });
                break;
        }

        NamedCommands.registerCommand("ArmState_Start", ArmCommands.updateSetpoint(arm, ArmStates.START));
        NamedCommands.registerCommand("ArmState_Intake", Commands.parallel(
                ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER), IntakeCommands.runIntake(intake, -6)));
        //NamedCommands.registerCommand("ArmState_L1", ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
        NamedCommands.registerCommand("ArmState_L2", Commands.parallel(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF), IntakeCommands.runIntake(intake, -2)));
        NamedCommands.registerCommand("ArmState_L4", Commands.parallel(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF),
                IntakeCommands.runIntake(intake, -3)));
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

    private void configureButtonBindings() {
        switch (Constants.currentDriver) {
            case MATT:
                drive.setDefaultCommand(
                        DriveCommands.joystickDrive(
                                drive,
                                () -> mattTranslation.StickYAxis() * 1.0,
                                () -> mattTranslation.StickXAxis() * 1.0,
                                () -> -mattRotation.StickXAxis() * 0.7,
                                driveMultiplier,
                                mattTranslation.fireStage1().or(mattTranslation.fireStage2())));

                mattTranslation.B1().onTrue(Commands.runOnce(robotState::zeroHeading));

                mattTranslation.A2().whileTrue(Commands.run(() -> drive.stopWithX(), drive));

                tractorController.button(21).onTrue((Commands.runOnce(() -> arm.updateSetpoint(ArmStates.CLIMB_STRETCH), arm)));
                
                tractorController.button(21).and(mattRotation.fireStage2()).whileTrue(
                        ClimbCommands.climbUp(climber))
                        .onFalse(ClimbCommands.stopClimber(climber));

                mattRotation.A2().onTrue(ClimbCommands.climbDown(climber))
                        .onFalse(ClimbCommands.stopClimber(climber));

                mattRotation.firePaddleUp().whileTrue(DriveCommands.pathFindToPose(() ->
                fieldUtil.reefPoses.get("Left" + Integer.toString(RobotState.getInstance().getReefSextant())), drive));
                mattTranslation.firePaddleUp().whileTrue(DriveCommands.pathFindToPose(() -> 
                fieldUtil.reefPoses.get("Right" + Integer.toString(RobotState.getInstance().getReefSextant())), drive));

                tractorController.button(9).onTrue(IntakeCommands.runIntake(intake, 6))
                        .onFalse(IntakeCommands.stopIntake(intake));

                tractorController.button(10).onTrue(Commands.parallel(
                        ArmCommands.updateSetpoint(arm, ArmStates.START), IntakeCommands.stopAll(intake)));

                // tractorController.button(10)
                //         .onTrue(Commands
                //                 .sequence(IntakeCommands.stopIntake(intake),
                //                         ArmCommands.updateSetpoint(arm, ArmStates.TRANSITION),
                //                         new WaitCommand(0.5/(ArmConstants.SHOULDER_MAX_VELOCITY_RPS)),
                //                         ArmCommands.updateSetpoint(arm, ArmStates.START)));

                tractorController.button(5)
                        .onTrue(Commands
                                .sequence(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER),
                                        IntakeCommands.runIntake(intake, -6)))
                        .onFalse(Commands
                                .sequence(IntakeCommands.stopIntake(intake),
                                        ArmCommands.updateSetpoint(arm, ArmStates.FEEDER_START_TRANSITION),
                                        new WaitCommand(0.5),
                                        ArmCommands.updateSetpoint(arm, ArmStates.START)));

                tractorController.button(4)
                        .onTrue(Commands
                                .sequence(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER_STRETCH),
                                        IntakeCommands.runIntake(intake, -6)))
                        .onFalse(Commands
                                .sequence(IntakeCommands.stopIntake(intake),
                                        ArmCommands.updateSetpoint(arm, ArmStates.FEEDER_START_TRANSITION),
                                        new WaitCommand(0.5),
                                        ArmCommands.updateSetpoint(arm, ArmStates.START)));

                tractorController.button(6).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
                
                tractorController.button(1)
                        .onTrue(Commands
                                .parallel(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF),
                                        IntakeCommands.runIntake(intake, 0)));

                tractorController.button(2)
                        .onTrue(Commands
                                .parallel(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L3_REEF),
                                        IntakeCommands.runIntake(intake, -3)));

                tractorController.button(3)
                        .onTrue(Commands
                                .parallel(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF),
                                        IntakeCommands.runIntake(intake, -3)));

                tractorController.button(12)
                .onTrue(ArmCommands.updateSetpoint(arm, ArmStates.PREP_L1_ALGAE))
                .onFalse(ArmCommands.updateSetpoint(arm, ArmStates.L1_ALGAE));

                tractorController.button(11)
                .onTrue(ArmCommands.updateSetpoint(arm, ArmStates.PREP_L2_ALGAE))
                .onFalse(ArmCommands.updateSetpoint(arm, ArmStates.L2_ALGAE));

                // tractorController.button(3)
                //         .onTrue(Commands
                //                 .sequence(IntakeCommands.stopIntake(intake),
                //                         IntakeCommands.runIntake(intake, -3),
                //                         ArmCommands.updateSetpoint(arm, ArmStates.REAR_L3_REEF),
                //                         new WaitCommand(0.3/(ArmConstants.SHOULDER_MAX_VELOCITY_RPS)),
                //                         ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF)));

                // Close valve and create vacuum pressure


                // X Postive is TOWARDS battery
                // Y positive is UP
                tractorController.button(18).onTrue(Commands.runOnce(()-> arm.bumpArmUsingArc(1), arm));
                // Bump up
                tractorController.button(17).onTrue((Commands.runOnce(()-> arm.bumpArmUsingArc(1), arm)));
                //Climber Prep
                //tractorController.button(22).onTrue((Commands.runOnce(() -> arm.updateSetpoint(ArmStates.FRONT_FEEDER_STRETCH), arm)));
                break;

                case ETHAN:
                        drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                        drive,
                                        () -> -ethanTranslation.StickYAxis(),
                                        () -> -ethanTranslation.StickXAxis(),
                                        () -> -ethanRotation.StickXAxis(),
                                        driveMultiplier,
                                        ethanTranslation.fireStage1().or(ethanTranslation.fireStage2())));

                        tractorController.button(21).onTrue((Commands.runOnce(() -> arm.updateSetpoint(ArmStates.CLIMB_STRETCH), arm)));
                        
                        tractorController.button(21).and(ethanRotation.fireStage2()).whileTrue(
                                ClimbCommands.climbUp(climber))
                                .onFalse(ClimbCommands.stopClimber(climber));

                        ethanRotation.A2().onTrue(ClimbCommands.climbDown(climber))
                                .onFalse(ClimbCommands.stopClimber(climber));

                        ethanTranslation.B1().onTrue(Commands.runOnce(robotState::zeroHeading));

                        ethanTranslation.A2().whileTrue(Commands.run(() -> drive.stopWithX(), drive));
                
                        ethanTranslation.firePaddleUp().whileTrue(DriveCommands.pathFindToPose(() ->
                        fieldUtil.reefPoses.get("Left" + Integer.toString(RobotState.getInstance().getReefSextant())), drive));
                        ethanRotation.firePaddleUp().whileTrue(DriveCommands.pathFindToPose(() -> 
                        fieldUtil.reefPoses.get("Right" + Integer.toString(RobotState.getInstance().getReefSextant())), drive));
                
                        tractorController.button(9).onTrue(IntakeCommands.runIntake(intake, 6))
                                        .onFalse(IntakeCommands.stopIntake(intake));
                
                        tractorController.button(10).onTrue(Commands.parallel(
                                ArmCommands.updateSetpoint(arm, ArmStates.START), IntakeCommands.stopAll(intake)));
                
                        tractorController.button(5)
                                .onTrue(Commands
                                        .sequence(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER),
                                        IntakeCommands.runIntake(intake, -6)))
                                .onFalse(Commands
                                        .sequence(IntakeCommands.stopIntake(intake),
                                        ArmCommands.updateSetpoint(arm, ArmStates.FEEDER_START_TRANSITION),
                                        new WaitCommand(0.5),
                                        ArmCommands.updateSetpoint(arm, ArmStates.START)));

                        tractorController.button(4)
                                .onTrue(Commands
                                        .sequence(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER_STRETCH),
                                        IntakeCommands.runIntake(intake, -6)))
                                .onFalse(Commands
                                        .sequence(IntakeCommands.stopIntake(intake),
                                        ArmCommands.updateSetpoint(arm, ArmStates.FEEDER_START_TRANSITION),
                                        new WaitCommand(0.5),
                                        ArmCommands.updateSetpoint(arm, ArmStates.START)));

                        tractorController.button(6).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
                
                        tractorController.button(1)
                                .onTrue(Commands
                                        .parallel(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF),
                                        IntakeCommands.runIntake(intake, 0)));

                        tractorController.button(2)
                                .onTrue(Commands
                                        .parallel(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L3_REEF),
                                        IntakeCommands.runIntake(intake, -3)));

                        tractorController.button(3)
                                .onTrue(Commands
                                        .parallel(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF),
                                                IntakeCommands.runIntake(intake, -3)));


                        // X Postive is TOWARDS battery
                        // Y positive is UP
                        tractorController.button(18).onTrue(Commands.runOnce(()-> arm.bumpArmUsingArc(1), arm));
                        // Bump up
                        tractorController.button(17).onTrue((Commands.runOnce(()-> arm.bumpArmUsingArc(1), arm)));

                        tractorController.button(22).onTrue((Commands.runOnce(() -> arm.updateSetpoint(ArmStates.FRONT_FEEDER_STRETCH), arm)));

                        tractorController.button(12)
                        .onTrue(ArmCommands.updateSetpoint(arm, ArmStates.PREP_L1_ALGAE))
                        .onFalse(ArmCommands.updateSetpoint(arm, ArmStates.L1_ALGAE));
        
                        tractorController.button(11)
                        .onTrue(ArmCommands.updateSetpoint(arm, ArmStates.PREP_L2_ALGAE))
                        .onFalse(ArmCommands.updateSetpoint(arm, ArmStates.L2_ALGAE));
                break;

            case PROGRAMMING:
                drive.setDefaultCommand(
                        DriveCommands.joystickDrive(
                                drive,
                                () -> programmingController.getLeftY(),
                                () -> programmingController.getLeftX(),
                                () -> -programmingController.getRightX(),
                                garrettDriveMultiplier,
                                programmingController.leftBumper()));

                climber.setDefaultCommand(ClimbCommands.driveClimber(climber, 
                                () -> programmingController.getRightTriggerAxis(),
                                () -> programmingController.getLeftTriggerAxis()));


                programmingController.button(8).onTrue(Commands.runOnce(robotState::zeroHeading));

                programmingController.a().onTrue(ArmCommands.updateSetpoint(arm,
                ArmStates.START));
                programmingController.b().onTrue(ArmCommands.updateSetpoint(arm,
                ArmStates.FRONT_FEEDER_STRETCH));
                programmingController.y().onTrue(ArmCommands.updateSetpoint(arm, 
                ArmStates.CLIMB_STRETCH));

                break;

            case MACBOOK:
                drive.setDefaultCommand(
                        DriveCommands.joystickDrive(
                                drive,
                                () -> programmingController.getRawAxis(1),
                                () -> programmingController.getRawAxis(0),
                                () -> -programmingController.getRawAxis(2),
                                driveMultiplier,
                                programmingController.leftTrigger()));

                programmingController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
                programmingController.button(12).onTrue(Commands.runOnce(robotState::zeroHeading));

                programmingController.button(1).whileTrue(DriveCommands.pathFindToPose(() ->
                fieldUtil.reefPoses.get("Left" + Integer.toString(RobotState.getInstance().getReefSextant())), drive));
                programmingController.button(2).whileTrue(DriveCommands.pathFindToPose(() -> 
                fieldUtil.reefPoses.get("Right" + Integer.toString(RobotState.getInstance().getReefSextant())), drive));
                // programmingController.button(1).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.START));
                // programmingController.button(2).onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_FEEDER));
                // programmingController.povDown().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L1_REEF));
                // programmingController.povRight().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.FRONT_L2_REEF));
                // programmingController.povUp().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L3_REEF));
                // programmingController.povLeft().onTrue(ArmCommands.updateSetpoint(arm, ArmStates.REAR_L4_REEF));

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
