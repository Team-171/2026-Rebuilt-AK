// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Aiming;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Intake intake = new Intake();

  private final Shooter shooter;

  private final Indexer indexer = new Indexer();

  private final Aiming aiming;

  private final Hopper hopper = new Hopper();

  private final Leds leds = new Leds();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  public final Joystick joystick = new Joystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        aiming = new Aiming(drive);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        shooter = new Shooter(drive);
      }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        aiming = new Aiming(drive);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        shooter = new Shooter(drive);
      }

      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        aiming = new Aiming(drive);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        shooter = new Shooter(drive);
      }
    }

    NamedCommands.registerCommand(
        "shoot", AutoCommands.shootCommand(shooter, aiming, indexer, hopper));
    NamedCommands.registerCommand("warmup", AutoCommands.warmUpShooter(shooter, aiming));
    NamedCommands.registerCommand("toggleIntake", AutoCommands.toggleIntake(intake));
    NamedCommands.registerCommand("intake", AutoCommands.intake(intake));
    NamedCommands.registerCommand(
        "shootWithAim",
        AutoCommands.aimAndShootAuto(shooter, aiming, indexer, hopper, drive, intake));
    NamedCommands.registerCommand("deployIntake", AutoCommands.deployIntakeAuto(intake));
    NamedCommands.registerCommand("stopIndex", AutoCommands.stopIndexer(indexer, hopper));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption(
        "Shoot (No Pathplanner)", AutoCommands.shootCommand(shooter, aiming, indexer, hopper));
    autoChooser.addOption(
        "aim and shoot",
        AutoCommands.aimAndShootAuto(shooter, aiming, indexer, hopper, drive, intake));

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

    // NamedCommands.registerCommands(
    //    AutoCommands.generateNamedCommands(shooter, aiming, intake, indexer, drive, hopper));

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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    /* drive.setDefaultCommand(
    DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightY(),
        () -> -controller.getRightX())); */
    // Reset gyro to 0° when back button is pressed
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.povUp().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Intake on left trigger
    controller.leftTrigger().whileTrue(IntakeCommands.intakeCommand(intake));
    // .onFalse(IntakeCommands.stopIntake(intake));

    // Aim and rev shooter on right trigger
    controller
        .rightTrigger()
        .whileTrue(
            DriveCommands.aimAndDrive(
                drive,
                aiming,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> aiming.getAngleToHub()))
        .whileTrue(shooter.shootWithDistanceCommand())
        .onFalse(shooter.stopShooterCommand());

    /* controller
    .rightTrigger()
    .whileTrue(
        DriveCommands.aimAndDrive(
            drive,
            aiming,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> aiming.getAngleToHub()))
    .whileTrue(ShooterCommands.shootCommand(shooter, aiming))
    .whileTrue(indexer.shootWithdelay())
    .whileTrue(hopper.shootWithDelay())
    .onFalse(ShooterCommands.stopHopper(hopper))
    .onFalse(ShooterCommands.stopIndexer(indexer))
    .onFalse(ShooterCommands.stopAiming(aiming))
    .onFalse(ShooterCommands.stopShoot(shooter)); */

    controller
        .a()
        .whileTrue(ShooterCommands.runHopper(hopper))
        .whileTrue(ShooterCommands.index(indexer))
        .onFalse(ShooterCommands.stopHopper(hopper))
        .onFalse(ShooterCommands.stopIndexer(indexer))
        .onFalse(ShooterCommands.stopAiming(aiming))
        .onFalse(ShooterCommands.stopShoot(shooter));

    controller
        .y()
        .whileTrue(hopper.reverseHopperCommand())
        .whileTrue(indexer.reverseIndexerCommand())
        .whileTrue(intake.reverseIntakeCommand())
        .onFalse(hopper.stopHopperCommand())
        .onFalse(indexer.stopIndexCommand())
        .onFalse(intake.stopIntakeWheelsCommand());

    controller.x().whileTrue(intake.intakeDeploy());
    controller.b().whileTrue(intake.intakeRetract());
    // controller.b().onTrue(intake.toggleIntake());

    /* controller
    .leftBumper()
    .onTrue(IntakeCommands.toggleIntake(intake)); */

    // passing

    controller
        .rightBumper()
        .whileTrue(shooter.shooterVelocity(50))
        .onFalse(shooter.stopShooterCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void teleopInit() {
    intake.stopAllIntakeMotors();
    hopper.stopHopper();
    shooter.stopShooter();
    indexer.stopIndex();
  }
}
