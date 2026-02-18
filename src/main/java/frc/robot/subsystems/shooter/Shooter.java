package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  SparkMax shooterMotor;

  public Shooter() {
    shooterMotor = new SparkMax(ShooterConstants.shooterCanId, MotorType.kBrushless);
  }

  public void shoot(double speed) {
    shooterMotor.set(speed);
  }

  @Override
  public void periodic() {}
}
