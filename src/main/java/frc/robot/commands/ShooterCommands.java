// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;

public class ShooterCommands {

  private ShooterCommands() {}

  public static Command shootCommand(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.shoot();
        },
        shooter);
  }

  public static Command stopShoot(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.stopShooter();
        },
        shooter);
  }

  public static Command index(Spindexer spindexer) {
    return Commands.run(
        () -> {
          spindexer.index();
        },
        spindexer);
  }

  public static Command stopIndexer(Spindexer spindexer) {
    return Commands.run(
        () -> {
          spindexer.stopIndex();
        },
        spindexer);
  }
}
