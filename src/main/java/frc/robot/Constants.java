package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static double MAX_SPEED = 10;
  public static double NathanSpeed = 2.0;
  public static final double TILT_ANGLE_OFFSET = -25;
  public static final double APRIL_TAG_OFFSET = 1.01237005816;

  public final class IntakeConstants {
    public static final int GROUND_PICKUP = 52;
    public static final int RIGHT_HOTWHEEL = 53;
    public static final int LEFT_HOTWHEEL = 54;
  }

  public final class DrumConstants {
    public static final int LOADING_DRUM = 43;
  }

  public final class TiltConstants {
    public static final int TILT = 44;
  }

  public final class ShooterConstants {
    public static final int TOP_SHOOTER = 42;
    public static final int BOTTOM_SHOOTER = 41;

    public static double SHOOTER_RPM = 60;

    // Shooter PID tuning - start with these values and adjust on dashboard
    // kP: Proportional gain - increase if response is slow, decrease if oscillating
    // kI: Integral gain - helps eliminate steady-state error (usually 0 or very small)
    // kD: Derivative gain - adds damping to prevent overshoot
    // FFkV: velocity gain - critical for velocity control, roughly 12V / max_rpm
    // FFkS: static gain - overcome static friction, smallest output that starts movement decreased
    // until mechanism holds still
    // FFkCos: cosine (arm) gravity gain - compensation for gravity in a rotary mechanism
    public static double shooterkP = 0.0001;
    public static double shooterkI = 0.0;
    public static double shooterkD = 0.0;
    public static double shooterFFkV = 0.22; // what ReCalc gave me
  }

  public final class ClimbConstants {
    public static final int RETIRED_CLIMB_A = 61;
    public static final int RETIRED_CLIMB_B = 62;
  }

  public final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public final class PowerConstants {
    public static final double INTAKE_POWER = 0.8;
    public static final double DRUM_POWER = 0.7;
    public static final double MAX_TILT_POWER = 1;
  }

  // vision still needs to be added
  public final class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
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

  public static class DashboardConstants {
    public static double tiltkP = 0.01;
    public static double tiltkI = 0.0;
    public static double tiltkD = 0.0;
    public static double tiltFFkS = 0.0;
    public static double tiltFFkV = 0.0;
    public static double tiltFFkCos = 0.0;

    public static void initDashboard() {
      // tilt PID values
      SmartDashboard.putNumber("Tilt/Tilt PID/kP", tiltkP);
      SmartDashboard.putNumber("Tilt/Tilt PID/kI", tiltkI);
      SmartDashboard.putNumber("Tilt/Tilt PID/kD", tiltkD);

      // tilt feedforward values
      SmartDashboard.putNumber("Tilt/Tilt FF/kS", tiltFFkS);
      SmartDashboard.putNumber("Tilt/Tilt FF/kV", tiltFFkV);
      SmartDashboard.putNumber("Tilt/Tilt FF/kCos", tiltFFkCos);
    }

    public static void updateFromDashboard() {
      tiltkP = SmartDashboard.getNumber("Tilt/Tilt PID/kP", tiltkP);
      tiltkI = SmartDashboard.getNumber("Tilt/Tilt PID/kI", tiltkI);
      tiltkD = SmartDashboard.getNumber("Tilt/Tilt PID/kD", tiltkD);
      tiltFFkS = SmartDashboard.getNumber("Tilt/Tilt FF/kS", tiltFFkS);
      tiltFFkV = SmartDashboard.getNumber("Tilt/Tilt FF/kV", tiltFFkV);
      tiltFFkCos = SmartDashboard.getNumber("Tilt/Tilt FF/kCos", tiltFFkCos);
    }
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }
}
