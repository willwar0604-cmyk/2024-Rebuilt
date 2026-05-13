package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Tilt extends SubsystemBase {
  SparkMax tilt = new SparkMax(Constants.TILT, MotorType.kBrushless);

  SparkClosedLoopController tiltController = tilt.getClosedLoopController();

  // range is 0-45 so safe range is 5-40
  LoggedNetworkNumber manualTargetAngle = new LoggedNetworkNumber("Tilt/ManualTargetAngle", 25.0);
  double targetAngle = 5.0;

  private double clampTargetAngle(double angle) {
    return Math.max(5.0, Math.min(40.0, angle));
  }

  public Tilt() {
    // tilt PID values
    LoggedNetworkNumber P = new LoggedNetworkNumber("Tilt/Tilt PID/kP", 0.02);
    LoggedNetworkNumber I = new LoggedNetworkNumber("Tilt/Tilt PID/kI", 0.0);
    LoggedNetworkNumber D = new LoggedNetworkNumber("Tilt/Tilt PID/kD", 0.0);

    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.idleMode(IdleMode.kBrake);

    tiltConfig
        .closedLoop
        .p(P.get())
        .i(I.get())
        .d(D.get())
        .maxMotion
        .cruiseVelocity(100)
        .maxAcceleration(200)
        .allowedProfileError(0.5);

    tiltConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(45)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(5);

    tilt.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setManualTiltAngle() {
    tiltController.setSetpoint(manualTargetAngle.get(), ControlType.kMAXMotionPositionControl);
  }

  public void setTiltAngle() {
    tiltController.setSetpoint(targetAngle, ControlType.kMAXMotionPositionControl);
  }

  public Command increaseTiltAngle() {
    return run(
        () -> {
          this.setTiltAngle();
          targetAngle = clampTargetAngle(targetAngle) + 0.25;
        });
  }

  public Command decreaseTiltAngle() {
    return run(
        () -> {
          this.setTiltAngle();
          targetAngle = clampTargetAngle(targetAngle) - .25;
        });
  }

  public Command joystickTilt(DoubleSupplier joystickY) {
    return run(
        () -> {
          this.setTiltAngle();
          double Y = joystickY.getAsDouble();
          targetAngle = clampTargetAngle(targetAngle + Y);
        });
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Tilt/Tilt Current Angle", tilt.getEncoder().getPosition());
    Logger.recordOutput("Tilt/Tilt Target Angle", targetAngle);
  }
}
