// Taken mainly from Charger Robotics #537

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class DashboardManager {
  private final RobotContainer robot;

  private final LoggedNetworkNumber matchTime = new LoggedNetworkNumber("Match Time");
  private final LoggedNetworkString match = new LoggedNetworkString("Match");
  private final LoggedNetworkString alliance = new LoggedNetworkString("Alliance");
  private final LoggedNetworkNumber periodTime = new LoggedNetworkNumber("Period Time");
  private final LoggedNetworkString timeframe = new LoggedNetworkString("Timeframe");
  private final LoggedDashboardChooser<Alliance> autoWinner;
  private final LoggedNetworkString activeHub = new LoggedNetworkString("Active Hub");
  private final Field2d field = new Field2d();

  public DashboardManager(RobotContainer robot) {
    this.robot = robot;

    SendableChooser<Alliance> autoWinnerChooser = new SendableChooser<>();
    autoWinnerChooser.addOption("Red", Alliance.Red);
    autoWinnerChooser.addOption("Blue", Alliance.Blue);
    this.autoWinner = new LoggedDashboardChooser<>("Auto Winner", autoWinnerChooser);

    SmartDashboard.putData("Swerve", robot.getDrive());
    SmartDashboard.putData("Field", field);
  }

  public void updateValues() {
    matchTime.set(DriverStation.getMatchTime());
    match.set(DriverStation.getMatchType().toString() + " " + DriverStation.getMatchNumber());

    Optional<Alliance> alliance = DriverStation.getAlliance();
    String color = "#cccccc";
    if (alliance.isPresent()) {
      color = getColorForAlliance(alliance.get());
    }
    this.alliance.set(color);

    field.setRobotPose(robot.getDrive().getPose());

    double periodTime = -1;
    String timeframe = "NONE";
    String activeHub = "#cccccc";
    if (DriverStation.isTeleop()) {
      double matchTime = DriverStation.getMatchTime();
      if (matchTime > secondsFromTimestamp(2, 10)) {
        // TRANSITION SHIFT (2:20-2:10)
        periodTime = matchTime - secondsFromTimestamp(2, 10);
        timeframe = "TRANSITION SHIFT";
        activeHub = getAllianceColorForShift(autoWinner.get(), 0);
      } else if (matchTime > secondsFromTimestamp(1, 45)) {
        // SHIFT 1 (2:10-1:45)
        periodTime = matchTime - secondsFromTimestamp(1, 45);
        timeframe = "SHIFT 1/4";
        activeHub = getAllianceColorForShift(autoWinner.get(), 1);
      } else if (matchTime > secondsFromTimestamp(1, 20)) {
        // SHIFT 2 (1:45-1:20)
        periodTime = matchTime - secondsFromTimestamp(1, 20);
        timeframe = "SHIFT 2/4";
        activeHub = getAllianceColorForShift(autoWinner.get(), 2);
      } else if (matchTime > secondsFromTimestamp(0, 55)) {
        // SHIFT 3 (1:20-0:55)
        periodTime = matchTime - secondsFromTimestamp(0, 55);
        timeframe = "SHIFT 3/4";
        activeHub = getAllianceColorForShift(autoWinner.get(), 3);
      } else if (matchTime > secondsFromTimestamp(0, 30)) {
        // SHIFT 4 (0:55-0:30)
        periodTime = matchTime - secondsFromTimestamp(0, 30);
        timeframe = "SHIFT 4/4";
        activeHub = getAllianceColorForShift(autoWinner.get(), 4);
      } else {
        // ENDGAME (0:30-0:00)
        periodTime = matchTime;
        timeframe = "ENDGAME";
        activeHub = getAllianceColorForShift(autoWinner.get(), 5);
      }
    }
    this.periodTime.set(periodTime);
    this.timeframe.set(timeframe);
    this.activeHub.set(activeHub);
  }

  private String getColorForAlliance(Alliance alliance) {
    return switch (alliance) {
      case Red -> "#f0120f";
      case Blue -> "#4047ed";
    };
  }

  private String getAllianceColorForShift(Alliance autoWinner, int shift) {
    // TRANSITION SHIFT or ENDGAME
    if (shift == 0 || shift == 5) return "#4beb3d";

    if (shift % 2 == 0) {
      return getColorForAlliance(autoWinner);
    } else {
      Alliance alliance =
          switch (autoWinner) {
            case Red -> Alliance.Blue;
            case Blue -> Alliance.Red;
          };
      return getColorForAlliance(alliance);
    }
  }

  private final double secondsFromTimestamp(int minutes, int seconds) {
    return minutes * 60 + seconds;
  }
}
