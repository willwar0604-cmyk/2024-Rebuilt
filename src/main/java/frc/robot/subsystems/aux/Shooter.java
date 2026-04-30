package frc.robot.subsystems.aux;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  SparkFlex topShooter =
      new SparkFlex(ShooterConstants.TOP_SHOOTER, SparkFlex.MotorType.kBrushless);
  SparkFlex bottomShooter =
      new SparkFlex(ShooterConstants.BOTTOM_SHOOTER, SparkFlex.MotorType.kBrushless);

  SparkClosedLoopController topShooterController = topShooter.getClosedLoopController();
  SparkClosedLoopController bottomShooterController = bottomShooter.getClosedLoopController();

  public Shooter() {
    SmartDashboard.putNumber("Shooter/Shooter RPM/RPM", ShooterConstants.SHOOTER_RPM);

    // shooter PID values
    SmartDashboard.putNumber("Shooter/Shooter PID/kP", ShooterConstants.shooterkP);
    SmartDashboard.putNumber("Shooter/Shooter PID/kI", ShooterConstants.shooterkI);
    SmartDashboard.putNumber("Shooter/Shooter PID/kD", ShooterConstants.shooterkD);
    SmartDashboard.putNumber("Shooter/Shooter PID/FFkV", ShooterConstants.shooterFFkV);

    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .p(ShooterConstants.shooterkP)
        .i(ShooterConstants.shooterkI)
        .d(ShooterConstants.shooterkD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .feedForward
        .kV(ShooterConstants.shooterFFkV);

    topShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void shooterRun() {
    // update shooter pid when shooting (hopefuly doesnt run too many times to overload memory)
    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .p(ShooterConstants.shooterkP)
        .i(ShooterConstants.shooterkI)
        .d(ShooterConstants.shooterkD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .feedForward
        .kV(ShooterConstants.shooterFFkV);

    topShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topShooterController.setSetpoint(ShooterConstants.SHOOTER_RPM, ControlType.kVelocity);
    bottomShooterController.setSetpoint(-ShooterConstants.SHOOTER_RPM, ControlType.kVelocity);
  }

  public void shooterStop() {
    topShooter.set(0);
    bottomShooter.set(0);
  }

  @Override
  public void periodic() {
    double dashboardRPM =
        SmartDashboard.getNumber("Shooter/Shooter RPM", ShooterConstants.SHOOTER_RPM);
    ShooterConstants.SHOOTER_RPM = Math.round(dashboardRPM);

    double dashboardP =
        SmartDashboard.getNumber("Shooter/Shooter PID/kP", ShooterConstants.shooterkP);
    double dashboardI =
        SmartDashboard.getNumber("Shooter/Shooter PID/kI", ShooterConstants.shooterkI);
    double dashboardD =
        SmartDashboard.getNumber("Shooter/Shooter PID/kD", ShooterConstants.shooterkD);
    double dashboardFFkV =
        SmartDashboard.getNumber("Shooter/Shooter PID/FFkV", ShooterConstants.shooterFFkV);
    ShooterConstants.shooterkP = dashboardP;
    ShooterConstants.shooterkD = dashboardD;
    ShooterConstants.shooterkI = dashboardI;
    ShooterConstants.shooterFFkV = dashboardFFkV;
  }
}
