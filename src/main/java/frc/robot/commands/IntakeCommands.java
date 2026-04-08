// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.CURRENTCOLOR;

public class IntakeCommands {

  private IntakeCommands() {}

  public static Command intakeCommand(Intake intake) {
    return Commands.run(
            () -> {
              intake.intake();
            },
            intake)
        .beforeStarting(() -> Leds.color = CURRENTCOLOR.BLUE);
  }

  public static Command stopIntake(Intake intake) {
    return Commands.run(
            () -> {
              intake.stopIntake();
            },
            intake)
        .finallyDo(() -> Leds.color = CURRENTCOLOR.TEAM_COLOR);
  }

  public static Command toggleIntake(Intake intake) {
    return Commands.either(
        intake.intakeRetract(), intake.intakeDeploy(), () -> intake.isIntakeDeployed());
  }
}
