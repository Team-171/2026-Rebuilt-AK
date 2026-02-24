// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class Aiming extends SubsystemBase {

  private final Drive drive;
  private static final InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

  // TODO: find the max speed of the robot for the constraints
  private final TrapezoidProfile.Constraints omegConstraints =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(500), Units.degreesToRadians(720));
  public final ProfiledPIDController pidControllerOmega =
      new ProfiledPIDController(
          ShooterConstants.shooterkP,
          ShooterConstants.shooterkI,
          ShooterConstants.shooterkD,
          omegConstraints);

  /** Creates a new aiming. */
  public Aiming(Drive drive) {
    this.drive = drive;
    // Values are placeholders only. this was the relationship between the distance of the robot and
    // the angle of the shooter
    // TODO: must find relationship between distance from target and velocity
    treeMap.put(1.526, 11.0);
    treeMap.put(1.82, 10.5);
    treeMap.put(2.16, 9.5);
    treeMap.put(2.5, 9.0);
    treeMap.put(2.82, 8.6);
    treeMap.put(3.135, 8.2);
    treeMap.put(3.5, 7.8);
    treeMap.put(3.85, 7.5);
    treeMap.put(4.29, 7.2);

    // treeMap.put(0.25, value);

    pidControllerOmega.reset(drive.getRotation().getRadians());
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));
    pidControllerOmega.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("angle", getAngleToHub());

    Logger.recordOutput("KP", ShooterConstants.shooterkP);
    Logger.recordOutput("distance to hub", getDisToHub());
    Logger.recordOutput("shooter power", getShooterPower());
  }

  // @AutoLogOutput
  public double getAngleToHub() {
    var alliance = DriverStation.getAlliance();

    double currentX = drive.getPose().getX();
    double currentY = drive.getPose().getY();
    double targetX;
    double targetY;

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      targetX = FieldConstants.Hub.topCenterPoint.getX();
      targetY = FieldConstants.Hub.topCenterPoint.getY();
    } else {
      targetX = AllianceFlipUtil.applyX(FieldConstants.Hub.topCenterPoint.getX());
      targetY = AllianceFlipUtil.applyY(FieldConstants.Hub.topCenterPoint.getY());
    }

    double calcX = targetX - currentX;
    double calcY = targetY - currentY;

    double result =
        pidControllerOmega.calculate(drive.getRotation().getRadians(), Math.atan2(calcY, calcX));

    return result;
  }

  public double getDisToHub() {
    Pose2d hubPose =
        new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), new Rotation2d());
    return PhotonUtils.getDistanceToPose(drive.getPose(), AllianceFlipUtil.apply(hubPose));
  }

  public Command resetPID() {
    return runOnce(() -> this.pidControllerOmega.reset(drive.getRotation().getRadians()));
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  private void stop() {}

  public double getShooterPower() {
    return treeMap.get(this.getDisToHub());
  }
}
