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

public class IntakeCommands {

  private IntakeCommands() {}

  public static Command intakeCommand(Intake intake) {
    return Commands.run(
        () -> {
          intake.intake();
        },
        intake);
  }

  public static Command stopIntake(Intake intake) {
    return Commands.run(
        () -> {
          intake.stopIntake();
        },
        intake);
  }

  public static Command toggleIntake(Intake intake) {
    return Commands.run(
        () -> {
          intake.toggleIntake();
        },
        intake);
  }
}
