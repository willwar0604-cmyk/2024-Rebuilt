package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  SparkFlex topShooter =
      new SparkFlex(ShooterConstants.TOP_SHOOTER, SparkFlex.MotorType.kBrushless);
  SparkFlex bottomShooter =
      new SparkFlex(ShooterConstants.BOTTOM_SHOOTER, SparkFlex.MotorType.kBrushless);

  SparkClosedLoopController topShooterController = topShooter.getClosedLoopController();
  SparkClosedLoopController bottomShooterController = bottomShooter.getClosedLoopController();

  public Shooter() {
    // shooter PID values
    LoggedNetworkNumber P = new LoggedNetworkNumber("Shooter/Shooter PID/kP", 0.0001);
    LoggedNetworkNumber I = new LoggedNetworkNumber("Shooter/Shooter PID/kI", 0.0);
    LoggedNetworkNumber D = new LoggedNetworkNumber("Shooter/Shooter PID/kD", 0.0);

    // shooter feedForward values
    LoggedNetworkNumber S = new LoggedNetworkNumber("Shooter/Shooter FF/kS", 0.0);
    LoggedNetworkNumber V = new LoggedNetworkNumber("Shooter/Shooter FF/kV", 0.0018);

    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .p(P.get())
        .i(I.get())
        .d(D.get())
        .feedForward
        .kS(S.get())
        .kV(V.get());

    topShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomShooter.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command shoot(Supplier<Double> RPM) {
    return run(() -> {
          topShooterController.setSetpoint(RPM.get(), ControlType.kVelocity);
          bottomShooterController.setSetpoint(RPM.get(), ControlType.kVelocity);
        })
        .finallyDo(() -> stop());
  }

  public void stop() {
    topShooter.set(0);
    bottomShooter.set(0);
  }

  public boolean isUpToSpeed() {
    return Math.abs(topShooter.getEncoder().getVelocity() - topShooterController.getSetpoint())
        <= 50;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Shooter/Shooter Current RPM/Top RPM", topShooter.getEncoder().getVelocity());
    Logger.recordOutput(
        "Shooter/Shooter Current RPM/Bottom RPM", bottomShooter.getEncoder().getVelocity());

    Logger.recordOutput("Shooter/Shooter RPM Setpoint", topShooterController.getSetpoint());

    Logger.recordOutput("Shooter/Shooter Is Up To Speed", isUpToSpeed());
  }
}
