package frc.robot;

public final class Constants {


  public static final class LimelightConstants {
    // Empty string = use the default limelight NetworkTables name.
    // If you renamed the camera in Limelight settings, put that name here.
    public static final String LIMELIGHT_NAME = "";

    // Pipeline to use for targeting.
    public static final int TARGET_PIPELINE = 0;

    // =========================================================
    // SEARCH / SEEK CONSTANTS
    // =========================================================

    // Turn speed used while no target is visible.
    // Too low = robot searches forever.
    // Too high = robot whips around and overshoots targets.
    public static final double SEARCH_TURN = 1.0;

    // Main proportional gain for turning to center the target using tx.
    // Turn output = tx * TURN_kP (+ TURN_MIN when needed).
    // Too low = lazy / slow aiming.
    // Too high = oscillation / twitching.
    public static final double TURN_kP = 0.02;

    // Minimum turn added whenever we are not yet aimed.
    // This helps overcome drivetrain friction.
    // Too low = robot may not move for small errors.
    // Too high = robot may bounce past center a lot.
    public static final double TURN_MIN = 0.10;

    // Clamp on turn output so it cannot spin too aggressively.
    public static final double TURN_MAX = 0.45;

    // Considered "aimed" when |tx| is below this many degrees.
    // Smaller = more precise but harder to finish.
    // Larger = easier to finish but less accurate.
    public static final double AIM_TOLERANCE_DEG = 1.5;

    // =========================================================
    // APPROACH / DISTANCE CONSTANTS
    // =========================================================

    // Proportional gain for driving forward/backward to target distance.
    // forward output = distance error * DRIVE_kP
    // Too low = robot crawls and may never quite get there.
    // Too high = robot lunges / overshoots.
    public static final double DRIVE_kP = 0.55;

    // Maximum forward speed while approaching.
    // Keep this modest while tuning.
    public static final double DRIVE_MAX_FWD = 0.40;

    // Maximum reverse speed if robot gets too close and needs to back up.
    // Usually keep this smaller than forward.
    public static final double DRIVE_MAX_REV = 0.25;

    // Robot will ONLY drive forward when it is roughly aimed.
    // This is very important.
    // If too small, robot may refuse to drive until perfectly centered.
    // If too large, robot may drive in while badly misaligned.
    public static final double DRIVE_ENABLE_TX_DEG = 6.0;

    // Stop when within this distance of the target setpoint.
    // Smaller = more precise but harder to settle.
    public static final double DISTANCE_TOLERANCE_METERS = 0.08;

    // =========================================================
    // DISTANCE GEOMETRY CONSTANTS
    // =========================================================
    // These MUST be measured on your real robot.
    // Distance math will be wrong if these are wrong.

    // Height of the Limelight lens center from the floor.
    public static final double LIMELIGHT_HEIGHT_METERS = 0.5;

    // Height of the target point from the floor.
    // Pick the actual point you want to drive toward.
    public static final double TARGET_HEIGHT_METERS = 1.0;

    // Limelight mounting angle above horizontal.
    // Example: if the camera points slightly upward, this might be 20 deg.
    public static final double LIMELIGHT_MOUNT_ANGLE_DEG = 20.0;

    // Desired final stopping distance from the target.
    // This is the "go to here" setpoint.
    public static final double TARGET_DISTANCE_METERS = 1.5;
  }

  private Constants() {}
}