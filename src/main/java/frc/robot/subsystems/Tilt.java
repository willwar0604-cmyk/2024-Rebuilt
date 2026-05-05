package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tilt extends SubsystemBase {
  SparkMax tilt = new SparkMax(Constants.TILT, MotorType.kBrushless);

  SparkClosedLoopController tiltController = tilt.getClosedLoopController();

  public double targetAngle = 190.0;
  public double currentAngle = tilt.getEncoder().getPosition() * 360;

  public static double P = 0.02;
  public static double I = 0.0;
  public static double D = 0.0;

  public Tilt() {
    // tilt PID values
    SmartDashboard.putNumber("Tilt/Tilt PID/kP", P);
    SmartDashboard.putNumber("Tilt/Tilt PID/kI", I);
    SmartDashboard.putNumber("Tilt/Tilt PID/kD", D);

    SmartDashboard.putNumber("Tilt/Tilt Target", targetAngle);

    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.idleMode(IdleMode.kCoast);

    tiltConfig
        .closedLoop
        .p(P)
        .i(I)
        .d(D)
        .maxMotion
        .cruiseVelocity(100)
        .maxAcceleration(200)
        .allowedProfileError(0.5);

    tiltConfig
        .softLimit
        .forwardSoftLimit(degreesToRotations(220.0))
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(degreesToRotations(190.0))
        .reverseSoftLimitEnabled(true);

    tilt.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private double degreesToRotations(double degrees) {
    return degrees / 360.0;
  }

  public void setTiltAngle() {
    if ((targetAngle > 190) || (targetAngle < 220)) {
      tiltController.setSetpoint(degreesToRotations(targetAngle), ControlType.kPosition);
    }
  }

  public void increaseTiltAngle() {
    if (Math.abs(currentAngle - targetAngle) > 10) {
      targetAngle += 1.0;
    }
    setTiltAngle();
  }

  public void decreaseTiltAngle() {
    if (Math.abs(currentAngle - targetAngle) > 10) {
      targetAngle -= 1.0;
    }
    setTiltAngle();
  }

  @Override
  public void periodic() {
    P = SmartDashboard.getNumber("Tilt/Tilt PID/kP", P);
    I = SmartDashboard.getNumber("Tilt/Tilt PID/kI", I);
    D = SmartDashboard.getNumber("Tilt/Tilt PID/kD", D);

    targetAngle = SmartDashboard.getNumber("Tilt/Tilt Target Angle", targetAngle);
    SmartDashboard.putNumber("Tilt/Tilt Current Angle", currentAngle);
  }
}
