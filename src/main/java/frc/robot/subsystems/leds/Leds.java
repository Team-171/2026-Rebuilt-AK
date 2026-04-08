package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {

  private final Spark ledStrip = new Spark(0);

  public static CURRENTCOLOR color = CURRENTCOLOR.TEAM_COLOR;

  public static enum CURRENTCOLOR {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    VIOLET,
    WHITE,
    TEAM_COLOR
  };

  public Leds() {}

  public void turnRed() {
    ledStrip.set(0.61);
  }

  public void turnGreen() {
    ledStrip.set(0.73);
  }

  public void turnBlue() {
    ledStrip.set(0.87);
  }

  public void turnYellow() {
    ledStrip.set(0.67);
  }

  public void turnViolet() {
    ledStrip.set(0.91);
  }

  public void turnWhite() {
    ledStrip.set(0.93);
  }

  public void teamColors() {
    ledStrip.set(0.51);
  }

  @Override
  public void periodic() {
    switch (color) {
      case RED -> turnRed();
      case GREEN -> turnGreen();
      case BLUE -> turnBlue();
      case YELLOW -> turnYellow();
      case VIOLET -> turnViolet();
      case WHITE -> turnWhite();
      case TEAM_COLOR -> teamColors();
      default -> teamColors();
    }

    Logger.recordOutput("Current LED Color", color);
  }
}
