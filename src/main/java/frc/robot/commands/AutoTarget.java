package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.HUB_POSITION;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoMath;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoTarget extends SequentialCommandGroup {
  public AutoTarget(
      Drive drive, Shooter shooter, Tilt tilt, Intake intake, CommandXboxController controller) {
    Supplier<Double> shooterSpeed =
        () -> AutoMath.getFuelSpeedToTarget(drive.getPose(), HUB_POSITION);

    Supplier<Rotation2d> targetYaw =
        () -> AutoMath.getRobotAngleToTarget(drive.getPose(), HUB_POSITION.toPose2d());

    Supplier<Double> correctedRobotYaw =
        () ->
            (Math.abs(
                drive.getRotation().getDegrees() > 0
                    ? Math.abs(drive.getRotation().getDegrees() - 180.0)
                    : Math.abs(drive.getRotation().getDegrees() + 180.0)));

    Supplier<Double> correctedTargetYaw =
        () ->
            Math.abs(
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                    ? flipAngle(targetYaw.get().getDegrees())
                    : targetYaw.get().getDegrees());

    Supplier<Double> yawError = () -> correctedTargetYaw.get() - correctedRobotYaw.get();

    addCommands(
        new ParallelCommandGroup(
                tilt.manualTilt(() -> 40.0),
                shooter.shoot(() -> shooterSpeed.get()),
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> targetYaw.get()),
                Commands.run(
                    () -> {
                      Logger.recordOutput("Targeting/Robot Yaw", correctedRobotYaw.get());
                      Logger.recordOutput("Targeting/Target Yaw", correctedTargetYaw.get());
                      Logger.recordOutput("Targeting/Angle Yaw", yawError.get());
                    }))
            .until(
                () -> (shooter.isUpToSpeed() && yawError.get() <= RobotConstants.ROTATION_ERROR)),
        new ParallelCommandGroup(
            shooter.shoot(() -> shooterSpeed.get()),
            intake.feed().onlyWhile(() -> shooter.isUpToSpeed()),
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> targetYaw.get())));
  }

  public static double flipAngle(double angle) {
    double reflectedAngle = -180 - angle;
    if (reflectedAngle < -180) {
      return reflectedAngle + 360;
    }
    return reflectedAngle;
  }
}
