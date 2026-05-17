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
import frc.robot.Constants.TiltConstants;
import frc.robot.VectorKit.hardware.AbsoluteEncoder;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Tilt extends SubsystemBase {
  private final SparkMax tilt = new SparkMax(TiltConstants.TILT, MotorType.kBrushless);

  private final SparkClosedLoopController tiltController = tilt.getClosedLoopController();

  // channel and offsets need to be double checked
  private final AbsoluteEncoder tiltEncoder = new AbsoluteEncoder(1, 195);

  public Tilt() {
    // tilt PID values
    LoggedNetworkNumber P = new LoggedNetworkNumber("Tilt/Tilt PID/kP", 0.02);
    LoggedNetworkNumber I = new LoggedNetworkNumber("Tilt/Tilt PID/kI", 0.0);
    LoggedNetworkNumber D = new LoggedNetworkNumber("Tilt/Tilt PID/kD", 0.0);

    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.idleMode(IdleMode.kCoast).closedLoop.p(P.get()).i(I.get()).d(D.get());

    tilt.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // range is 0-45 so safe range is 10-40
  private double clampedAngle(double unclampedAngle) {
    return Math.max(10.0, Math.min(40.0, unclampedAngle));
  }

  public Command joystickTilt(Supplier<Double> deltaAngle) {
    return run(
        () -> {
          tiltController.setSetpoint(
              clampedAngle((tiltEncoder.getRaw() * 360) + deltaAngle.get()), ControlType.kPosition);
        });
  }

  public Command manualTilt(Supplier<Double> manualAngle) {
    return joystickTilt(() -> manualAngle.get() - (tiltEncoder.getRaw() * 360));
  }

  @Override
  public void periodic() {

    if ((tiltEncoder.getRaw() * 360) >= 40) {
      tiltController.setSetpoint(40, ControlType.kPosition);
    }

    if ((tiltEncoder.getRaw() * 360) <= 10) {
      tiltController.setSetpoint(10, ControlType.kPosition);
    }

    Logger.recordOutput("Tilt/Tilt Current Rotation", tiltEncoder.get());
    Logger.recordOutput("Tilt/Tilt Current Raw Angle", tiltEncoder.getRaw() * 360);
    Logger.recordOutput("Tilt/Tilt Setpoint", tiltController.getSetpoint());
  }
}
