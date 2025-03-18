// Copyright 2021-2025 FRC 6328
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSparkMax;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberSparkMax;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorModule;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator m_elevator = new Elevator(new ElevatorModule());
  private final CoralIntake m_coralIntake = new CoralIntake(new CoralIntakeSparkMax());
  private final AlgaeIntake m_algaeIntake = new AlgaeIntake(new AlgaeIntakeSparkMax());
  private final Climber m_climber = new Climber(new ClimberSparkMax());

  // Controller
  private final CommandXboxController Dcontroller = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand(
        "L2",
        new SequentialCommandGroup(
            m_elevator.elevatorToLevel2().alongWith(m_coralIntake.turntoNeutral()).withTimeout(1)));

    NamedCommands.registerCommand("shoot coral", m_coralIntake.outtakeCoral());

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                new VisionIOLimelight());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                //new VisionIOSim());
                new VisionIOLimelight());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
               // new VisionIO() {});
               new VisionIOLimelight());
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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> Dcontroller.getLeftY(),
            () -> Dcontroller.getLeftX(),
            () -> -Dcontroller.getRightX(),
            () -> Dcontroller.getRightTriggerAxis()));

    // Dcontroller.rightTrigger().whileTrue(DriveCommands.slowMode(drive));

    // //when bottom on Dpad is pressed, the level 0 sequence is run
    m_operatorController.povDown().onTrue(L0);

    // when left on Dpad is pressed, the level 1 sequence is run
    m_operatorController.povLeft().onTrue(L1);

    // when right on Dpad is pressed, the level 2 sequence is run
    m_operatorController.povRight().onTrue(L2);

    // when top on Dpad is pressed, the level 3 sequence is run
    m_operatorController.povUp().onTrue(L3);

    // when the Y button is held down, the elevator is set to level 2.55 and the coral intake is set
    // to pivot position 5
    m_operatorController
        .y()
        .onTrue(
            m_elevator
                .setElevatorPosition(4.5)
                .alongWith(m_coralIntake.setPivotPosition(5.1).withTimeout(1)));

    // when the A button is held down, the elevator is set to level 0 and the coral intake is set to
    // pivot position 0
    m_operatorController
        .a()
        .onTrue(
            m_elevator
                .setElevatorPosition(ElevatorConstants.kElevatorLevel2 + 0.5)
                .alongWith(m_coralIntake.turntoNeutral())
                .alongWith(m_algaeIntake.turntoNeutral()));

    // when the left bumper is held down, the algae intake motor spins to intake the algae
    m_operatorController.leftBumper().whileTrue(m_algaeIntake.intakeAlgae());

    // when the right bumper is held down, the algae intake motor spins to outtake the algae
    m_operatorController.rightBumper().whileTrue(m_algaeIntake.outtakeAlgae());

    // when the left trigger is held down, the coral intake motor spins to intake the coral
    m_operatorController.leftTrigger().whileTrue(m_coralIntake.intakeCoral());

    // when the right trigger is held down, the coral intake motor spins to outtake the coral
    m_operatorController.rightTrigger().whileTrue(m_coralIntake.outtakeCoral());

    Dcontroller.a().whileTrue(m_climber.climbDown());
    Dcontroller.y().whileTrue(m_climber.climbUp());

    // Dcontroller.x().onTrue(drive.alignToReef());
    // Dcontroller.b().onTrue(drive.alignToReef());

    // when the start button (button with the 3 lines) is held down, the coral pivot motor spins to
    // move the pivot downwards
    m_operatorController
        .start()
        .and(m_operatorController.b())
        .whileTrue(m_algaeIntake.turntoDown());

    // when the back button (button with the 3 lines) is held down, the coral pivot motor spins to
    // move the pivot upwards
    m_operatorController.start().and(m_operatorController.x()).whileTrue(m_algaeIntake.turntoUp());

    // sets the pivot position for intaking the coral from the player position
    m_operatorController.x().onTrue(m_coralIntake.setPivotPosition(4.5));
  }
  // sequantial command group for level 0 sco(ring, scores the corala and then brings elevator back
  // to 0

  SequentialCommandGroup L0 =
      new SequentialCommandGroup(
          m_elevator.elevatorToLevel0().alongWith(m_coralIntake.turntoNeutral()).withTimeout(1)
          // m_coralIntake.outtakeCoral().withTimeout(1.5),
          // m_elevator.resetElevatorPosition()
          );

  // sequantial command group for level 1 scoring, scores the corala and then brings elevator back
  // to 0
  SequentialCommandGroup L1 =
      new SequentialCommandGroup(
          m_elevator.elevatorToLevel1().alongWith(m_coralIntake.turntoNeutral()).withTimeout(1)
          // m_coralIntake.outtakeCoral().withTimeout(1.5),
          // m_elevator.resetElevatorPosition()
          );

  // sequantial command group for level 2 scoring, scores the corala and then brings elevator back
  // to 0
  SequentialCommandGroup L2 =
      new SequentialCommandGroup(
          m_elevator.elevatorToLevel2().alongWith(m_coralIntake.turntoNeutral()).withTimeout(1)
          // m_coralIntake.outtakeCoral().withTimeout(1.5),
          // m_elevator.resetElevatorPosition()
          );

  // sequantial command group for level 3 scoring, scores the corala and then brings elevator back
  // to 0
  SequentialCommandGroup L3 =
      new SequentialCommandGroup(
          m_elevator
              .elevatorToLevel3()
              .alongWith(m_coralIntake.setPivotPosition(2.8))
              .withTimeout(1)
          // m_coralIntake.outtakeCoral().withTimeout(1.5),
          // m_elevator.resetElevatorPosition()
          );

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
