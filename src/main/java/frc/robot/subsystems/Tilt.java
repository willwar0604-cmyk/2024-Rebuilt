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
  static SparkMax tilt = new SparkMax(Constants.TILT, MotorType.kBrushless);

  static SparkClosedLoopController tiltController = tilt.getClosedLoopController();

  // range is 0-45 so safe range is 10-40
  static double targetAngle = 10.0;

  private double clampTargetAngle(double angle) {
    return Math.max(7.5, Math.min(40.0, angle));
  }

  public Tilt() {
    // tilt PID values
    LoggedNetworkNumber P = new LoggedNetworkNumber("Tilt/Tilt PID/kP", 0.02);
    LoggedNetworkNumber I = new LoggedNetworkNumber("Tilt/Tilt PID/kI", 0.0);
    LoggedNetworkNumber D = new LoggedNetworkNumber("Tilt/Tilt PID/kD", 0.0);

    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.idleMode(IdleMode.kCoast);

    tiltConfig.closedLoop.p(P.get()).i(I.get()).d(D.get());

    tilt.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public static void resetTiltPosition() {
    tilt.getEncoder().setPosition(0);
  }

  public void setTiltAngle() {
    tiltController.setSetpoint(
        targetAngle + 0.5,
        ControlType.kPosition); // add 0.5 bc it tends to be half a degree too low,
    // the bouncing looks like play/lash in the mounting/gears, the actual encoder value barely
    // changes so no programming fix
  }

  public Command intakeTiltFix() {
    return runOnce(
        () -> {
          targetAngle = 10;
        });
  }

  public Command joystickTilt(DoubleSupplier joystickY) {
    return run(
        () -> {
          double Y = joystickY.getAsDouble();
          if (Math.abs(Y) > 0.150) {
            targetAngle = clampTargetAngle(targetAngle + Math.pow(Y, 3));
          }
        });
  }

  @Override
  public void periodic() {
    if (Math.abs(tilt.getEncoder().getPosition() - targetAngle) >= 1) {
      setTiltAngle();
    }

    Logger.recordOutput("Tilt/Tilt Current Angle", tilt.getEncoder().getPosition());
    Logger.recordOutput("Tilt/Tilt Target Angle", targetAngle);
  }
}
