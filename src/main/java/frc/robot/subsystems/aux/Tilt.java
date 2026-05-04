package frc.robot.subsystems.aux;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Tilt extends SubsystemBase {
  SparkMax tilt = new SparkMax(TiltConstants.TILT, MotorType.kBrushless);

  SparkClosedLoopController tiltController = tilt.getClosedLoopController();

  double targetAngle = 190.0;

  public Tilt() {
    // tilt PID values
    SmartDashboard.putNumber("Tilt/Tilt PID/kP", TiltConstants.tiltkP);
    SmartDashboard.putNumber("Tilt/Tilt PID/kI", TiltConstants.tiltkI);
    SmartDashboard.putNumber("Tilt/Tilt PID/kD", TiltConstants.tiltkD);

    SmartDashboard.putNumber("Tilt/Tilt Target", targetAngle);

    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.idleMode(IdleMode.kCoast);

    tiltConfig
        .closedLoop
        .p(TiltConstants.tiltkP)
        .i(TiltConstants.tiltkI)
        .d(TiltConstants.tiltkD)
        .outputRange(-1, 1)
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

  public void setTiltAngle() {
    tiltController.setSetpoint(degreesToRotations(targetAngle), ControlType.kPosition);
  }

  public Command setTiltAngleCommand() {
    return this.runOnce(this::setTiltAngle);
  }

  private double degreesToRotations(double degrees) {
    return degrees / 360.0;
  }

  @Override
  public void periodic() {
    TiltConstants.tiltkP = SmartDashboard.getNumber("Tilt/Tilt PID/kP", TiltConstants.tiltkP);
    TiltConstants.tiltkI = SmartDashboard.getNumber("Tilt/Tilt PID/kI", TiltConstants.tiltkI);
    TiltConstants.tiltkD = SmartDashboard.getNumber("Tilt/Tilt PID/kD", TiltConstants.tiltkD);

    targetAngle = SmartDashboard.getNumber("Tilt/Tilt Target", targetAngle);
    SmartDashboard.putNumber("Tilt/Tilt Current", tilt.getEncoder().getPosition() * 360.0);
  }
}
