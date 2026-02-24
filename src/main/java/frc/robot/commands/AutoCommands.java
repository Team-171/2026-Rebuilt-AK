package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;

public class AutoCommands {
  private AutoCommands() {}

  public static Command shootCommand() {
    return null;
  }

  public static Command aimCommand() {
    return null;
  }

  public static Command indexCommand() {
    return null;
  }

  public static List<Pair<String, Command>> generateNamedCommands() {
    List<Pair<String, Command>> list = new ArrayList<>();

    list.add(new Pair<>("shoot", shootCommand()));
    list.add(new Pair<>("aim", aimCommand()));

    return list;
  }
}
