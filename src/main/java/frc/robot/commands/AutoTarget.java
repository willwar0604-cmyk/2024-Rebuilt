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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoTarget extends SequentialCommandGroup {

  LoggedNetworkNumber shooterMultiplier =
      new LoggedNetworkNumber("Shooter/Shooter Auto Multiplier", 100);

  public AutoTarget(
      Drive drive, Shooter shooter, Tilt tilt, Intake intake, CommandXboxController controller) {
    Supplier<Double> shooterSpeed =
        () ->
            shooterMultiplier.get() * AutoMath.getFuelSpeedToTarget(drive.getPose(), HUB_POSITION);

    Supplier<Double> correctedShooterSpeed = () -> Math.min(2500.0, shooterSpeed.get());

    Supplier<Rotation2d> targetYaw =
        () -> AutoMath.getRobotAngleToTarget(drive.getPose(), HUB_POSITION.toPose2d());

    Supplier<Rotation2d> flippedTargetYaw =
        () -> Rotation2d.fromDegrees(targetYaw.get().getDegrees() + 180);

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
                    ? targetYaw.get().getDegrees()
                    : flipAngle(targetYaw.get().getDegrees()));

    Supplier<Double> yawError = () -> correctedTargetYaw.get() - correctedRobotYaw.get();

    addCommands(
        new ParallelCommandGroup(
                tilt.manualTilt(() -> 40.0),
                shooter.shoot(() -> correctedShooterSpeed.get()),
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> controller.getLeftY(),
                    () -> controller.getLeftX(),
                    () -> flippedTargetYaw.get()),
                Commands.run(
                    () -> {
                      Logger.recordOutput("Targeting/Robot Yaw", correctedRobotYaw.get());
                      Logger.recordOutput("Targeting/Target Yaw", correctedTargetYaw.get());
                      Logger.recordOutput("Targeting/Angle Yaw", yawError.get());
                    }))
            .until(
                () -> (shooter.isUpToSpeed() && yawError.get() <= RobotConstants.ROTATION_ERROR)),
        new ParallelCommandGroup(
            shooter.shoot(() -> correctedShooterSpeed.get()),
            intake.feed().onlyWhile(() -> shooter.isUpToSpeed()),
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> flippedTargetYaw.get())));
  }

  public static double flipAngle(double angle) {
    double reflectedAngle = -180 - angle;
    if (reflectedAngle < -180) {
      return reflectedAngle + 360;
    }
    return reflectedAngle;
  }
}
