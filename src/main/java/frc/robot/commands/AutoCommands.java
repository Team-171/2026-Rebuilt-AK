package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Aiming;
import frc.robot.subsystems.shooter.Shooter;
import java.util.ArrayList;
import java.util.List;

public class AutoCommands {

  private AutoCommands() {}

  public static Command shootCommand(
      Shooter shooter, Aiming aiming, Indexer indexer, Hopper hopper) {
    return Commands.parallel(
        shooter.shootWithDistanceCommand(),
        // new WaitCommand(1),
        // ShooterCommands.shootCommand(shooter, aiming),
        ShooterCommands.index(indexer),
        ShooterCommands.runHopper(hopper));
  }

  public static Command shootCommandBetter(Indexer indexer, Hopper hopper) {
    return Commands.parallel(indexer.shootWithOUTDelayCommand(), hopper.shootWithOUTDelay());
  }

  public static Command warmUpShooter(Shooter shooter, Aiming aiming) {
    // return ShooterCommands.shootCommand(shooter, aiming);
    return shooter.shootWithDistanceCommand();
  }

  public static Command toggleIntake(Intake intake) {
    return IntakeCommands.toggleIntake(intake);
  }

  public static Command intake(Intake intake) {
    return IntakeCommands.intakeCommand(intake);
  }

  public static Command aimAndShootAuto(
      Shooter shooter, Aiming aiming, Indexer indexer, Hopper hopper, Drive drive, Intake intake) {
    return Commands.parallel(
        shooter.shootWithDistanceCommand(),
        DriveCommands.aimAndDrive(drive, aiming, () -> 0, () -> 0, () -> aiming.getAngleToHub()),
        /* intake.intakeCommand(),
        intake.intakeDeploy(), */
        new SequentialCommandGroup(new WaitCommand(1), shootCommandBetter(indexer, hopper)));
  }

  public static Command stopIndexer(Indexer indexer, Hopper hopper) {
    return Commands.parallel(indexer.stopIndexCommand(), hopper.stopHopperCommand());
  }

  public static Command deployIntakeAuto(Intake intake) {
    return Commands.sequence(intake.intakeDeploy().withTimeout(1), intake.intakeCommand())
        .withTimeout(2);
  }

  public static List<Pair<String, Command>> generateNamedCommands(
      Shooter shooter, Aiming aiming, Intake intake, Indexer indexer, Drive drive, Hopper hopper) {
    List<Pair<String, Command>> list = new ArrayList<>();

    list.add(new Pair<>("shoot", shootCommand(shooter, aiming, indexer, hopper)));
    list.add(new Pair<>("warmup", warmUpShooter(shooter, aiming)));
    list.add(new Pair<>("toggleIntake", toggleIntake(intake)));
    list.add(new Pair<>("intake", intake(intake)));

    return list;
  }
}
