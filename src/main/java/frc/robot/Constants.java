package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static class IntakeConstants {
    public static final int GROUND_PICKUP = 52;
    public static final int RIGHT_HOTWHEEL = 53;
    public static final int LEFT_HOTWHEEL = 54;
    public static final int LOADING_DRUM = 43;
  }

  public static class TiltConstants {
    public static final int TILT = 44;
  }

  public static class ShooterConstants {
    public static final int TOP_SHOOTER = 42;
    public static final int BOTTOM_SHOOTER = 41;

    public static final double GRAVITATIONAL_CONSTANT = 9.8; // m/s
  }

  public static class ClimbConstants {
    public static final int RETIRED_CLIMB_A = 61;
    public static final int RETIRED_CLIMB_B = 62;
  }

  public static final int DRIVER_CONTROLLER_PORT = 0;

  public static class VisionConstants {
    // Apriltag Field Layout

    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double FIELD_WIDTH = 16.541;
    public static final double FIELD_HEIGHT = 8.069;

    /***************************************************
     * All camera transforms are taken straight from the
     * original 2024 code and most likely need updating
     ***************************************************/

    // Name of PhotonVision Camera
    public static final String LeftCam = "Camera 0";

    // Position of PhotonVision Camera
    public static final Transform3d robotToLeftCam =
        new Transform3d(
            Units.inchesToMeters(-0.34), // X
            Units.inchesToMeters(0.22), // Y
            Units.inchesToMeters(0.20), // Z
            new Rotation3d(
                Math.toRadians(0), // Roll
                Math.toRadians(33), // Pitch
                Math.toRadians(160))); // Yaw

    // Name of PhotonVision Camera
    public static final String RightCam = "Camera 1";

    // Position of PhotonVision Camera
    public static final Transform3d robotToRightCam =
        new Transform3d(
            Units.inchesToMeters(-0.04), // X
            Units.inchesToMeters(-0.22), // Y
            Units.inchesToMeters(0.20), // Z
            new Rotation3d(
                Math.toRadians(0), // Roll
                Math.toRadians(-33), // Pitch
                Math.toRadians(200))); // Yaw

    // Basic filtering threshholds
    public static final double MAX_AMBIGUITY = 0.2;
    public static final double MAX_Z_ERROR = 0.2;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.03; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static class FieldConstants {
    public static final Pose3d HUB_POSITION =
        new Pose3d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(158.84),
            Units.inchesToMeters(70),
            new Rotation3d()); // Change 5.9 to 70" (2" below top of target)
  }
}
