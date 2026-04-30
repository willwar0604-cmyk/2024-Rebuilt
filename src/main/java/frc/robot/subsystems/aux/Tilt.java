package frc.robot.subsystems.aux;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tilt extends SubsystemBase {
  SparkMax tilt = new SparkMax(Constants.TiltConstants.TILT, MotorType.kBrushless);

  SparkClosedLoopController tiltController = tilt.getClosedLoopController();

  public Tilt() {
    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .p(Constants.DashboardConstants.tiltkP)
        .i(Constants.DashboardConstants.tiltkI)
        .d(Constants.DashboardConstants.tiltkD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1)
        .feedForward
        .kV(Constants.DashboardConstants.tiltFFkV);

    // MAXMotion position control for arms seems... annoying at best

    tilt.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*********************************************************
   * I havent done anything with this yet because honestly
   * im terrified of accidentally breaking something, maybe
   * ill test with one of the shooter rollers or smt that
   * wont break something if i do it wrong first
   *********************************************************/

  public void tiltUp() {}

  public void tiltDown() {}
}
