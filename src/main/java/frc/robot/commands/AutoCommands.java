package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Aiming;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import java.util.ArrayList;
import java.util.List;

public class AutoCommands {

  private AutoCommands() {}

  public static Command shootCommand(Shooter shooter, Aiming aiming) {
    return Commands.race(ShooterCommands.shootCommand(shooter, aiming), new WaitCommand(2));
  }

  public static Command aimCommand(Aiming aiming, Drive drive) {
    // return Commands.race(DriveCommands)
    return null;
  }

  public static Command indexCommand(Spindexer indexer) {
    return null;
  }

  public static List<Pair<String, Command>> generateNamedCommands(
      Shooter shooter, Aiming aiming, Intake intake, Spindexer spindexer, Drive drive) {
    List<Pair<String, Command>> list = new ArrayList<>();

    list.add(new Pair<>("shoot", shootCommand(shooter, aiming)));
    list.add(new Pair<>("aim", aimCommand(aiming, drive)));
    list.add(new Pair<>("index", indexCommand(spindexer)));

    return list;
  }
}
