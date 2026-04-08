// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.CURRENTCOLOR;
import frc.robot.subsystems.shooter.Aiming;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {

  private ShooterCommands() {}

  public static Command shootCommand(Shooter shooter, Aiming aiming) {
    return Commands.run(
            () -> {
              shooter.shooterVelocity(aiming.getShooterPower());
              Logger.recordOutput("shooting", true);
            },
            shooter)
        .beforeStarting(() -> Leds.color = CURRENTCOLOR.GREEN);
  }

  public static Command shootConstantCommand(Shooter shooter, DoubleSupplier speed) {
    return Commands.run(
        () -> {
          shooter.shoot(speed.getAsDouble());
          Logger.recordOutput("shooting", true);
        },
        shooter);
  }

  public static Command stopShoot(Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.stopShooter();
              Logger.recordOutput("shooting", false);
            },
            shooter)
        .finallyDo(() -> Leds.color = CURRENTCOLOR.TEAM_COLOR);
  }

  public static Command index(Indexer spindexer) {
    return Commands.run(
        () -> {
          spindexer.index();
        },
        spindexer);
  }

  public static Command stopIndexer(Indexer spindexer) {
    return Commands.run(
        () -> {
          spindexer.stopIndex();
        },
        spindexer);
  }

  public static Command stopAiming(Aiming aiming) {
    return aiming.stopCommand();
  }

  public static Command runHopper(Hopper hopper) {
    return Commands.run(
        () -> {
          hopper.moveHopper();
        },
        hopper);
  }

  public static Command stopHopper(Hopper hopper) {
    return Commands.run(
        () -> {
          hopper.stopHopper();
        },
        hopper);
  }
}
