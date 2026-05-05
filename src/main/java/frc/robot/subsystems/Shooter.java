package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  SparkFlex topShooter = new SparkFlex(Constants.TOP_SHOOTER, SparkFlex.MotorType.kBrushless);
  SparkFlex bottomShooter = new SparkFlex(Constants.BOTTOM_SHOOTER, SparkFlex.MotorType.kBrushless);

  SparkClosedLoopController topShooterController = topShooter.getClosedLoopController();
  SparkClosedLoopController bottomShooterController = bottomShooter.getClosedLoopController();

  public static double SHOOTER_RPM = 1000;
  public static double P = 0.0001;
  public static double I = 0.0;
  public static double D = 0.0;
  public static double S = 0.0;
  public static double V = 0.0018;

  public Shooter() {
    SmartDashboard.putNumber("Shooter/Shooter RPM/RPM", SHOOTER_RPM);

    // shooter PID values
    SmartDashboard.putNumber("Shooter/Shooter PID/kP", P);
    SmartDashboard.putNumber("Shooter/Shooter PID/kI", I);
    SmartDashboard.putNumber("Shooter/Shooter PID/kD", D);
    SmartDashboard.putNumber("Shooter/Shooter PID/FFkV", V);

    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig.idleMode(IdleMode.kBrake).closedLoop.p(P).i(I).d(D).feedForward.kS(S).kV(V);

    topShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command spinUp() {
    return runOnce(
        () -> {
          topShooterController.setSetpoint(SHOOTER_RPM, ControlType.kVelocity);
          bottomShooterController.setSetpoint(SHOOTER_RPM, ControlType.kVelocity);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          topShooterController.setSetpoint(0, ControlType.kVelocity);
          bottomShooterController.setSetpoint(0, ControlType.kVelocity);
        });
  }

  public boolean isUpToSpeed() {
    return Math.abs(topShooter.getEncoder().getVelocity() - SHOOTER_RPM) <= 50;
  }

  @Override
  public void periodic() {
    SHOOTER_RPM = Math.round(SmartDashboard.getNumber("Shooter/Shooter RPM", SHOOTER_RPM));

    P = SmartDashboard.getNumber("Shooter/Shooter PID/kP", P);
    I = SmartDashboard.getNumber("Shooter/Shooter PID/kI", I);
    D = SmartDashboard.getNumber("Shooter/Shooter PID/kD", D);
    V = SmartDashboard.getNumber("Shooter/Shooter PID/FFkV", V);

    SmartDashboard.putNumber(
        "Shooter/Shooter Current RPM/Top RPM", topShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber(
        "Shooter/Shooter Current RPM/Bottom RPM", bottomShooter.getEncoder().getVelocity());
  }
}
