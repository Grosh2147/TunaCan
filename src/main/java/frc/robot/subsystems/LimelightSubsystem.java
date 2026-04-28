package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
private final String limelightName = Constants.LimelightConstants.LIMELIGHT_NAME;  private static final double INVALID_DISTANCE = -1.0;

  // Debug values for dashboard / Elastic
  private double distanceErrorMeters = 0.0;
  private double approachForward = 0.0;
  private double approachTurn = 0.0;

  // 0 = idle, 1 = seeking, 2 = aiming, 3 = driving, 4 = ready
  private double visionState = 0.0;

  public LimelightSubsystem() {
    setTargetPipeline();
  }

  public void setTargetPipeline() {
    LimelightHelpers.setPipelineIndex(
        limelightName,
        Constants.LimelightConstants.TARGET_PIPELINE);
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(limelightName);
  }

  public double getTx() {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTy() {
    return LimelightHelpers.getTY(limelightName);
  }

  public double getTa() {
    return LimelightHelpers.getTA(limelightName);
  }

  public double getDistanceMeters() {
    if (!hasTarget()) {
      return INVALID_DISTANCE;
    }

    double totalAngleDeg =
        Constants.LimelightConstants.LIMELIGHT_MOUNT_ANGLE_DEG + getTy();
    double totalAngleRad = Math.toRadians(totalAngleDeg);
    double tanValue = Math.tan(totalAngleRad);

    if (Math.abs(tanValue) < 1e-6) {
      return INVALID_DISTANCE;
    }

    double distance =
        (Constants.LimelightConstants.TARGET_HEIGHT_METERS
            - Constants.LimelightConstants.LIMELIGHT_HEIGHT_METERS)
        / tanValue;

    if (!Double.isFinite(distance) || distance < 0.0) {
      return INVALID_DISTANCE;
    }

    return distance;
  }

  public boolean hasValidDistance() {
    return getDistanceMeters() != INVALID_DISTANCE;
  }

  public void setApproachDebug(
      double distanceErrorMeters,
      double approachForward,
      double approachTurn) {
    this.distanceErrorMeters = distanceErrorMeters;
    this.approachForward = approachForward;
    this.approachTurn = approachTurn;
  }

  public void setVisionState(double visionState) {
    this.visionState = visionState;
  }

  public boolean isIdle() {
    return visionState == 0.0;
  }

  public boolean isSeeking() {
    return visionState == 1.0;
  }

  public boolean isAiming() {
    return visionState == 2.0;
  }

  public boolean isDriving() {
    return visionState == 3.0;
  }

  public boolean isReady() {
    return visionState == 4.0;
  }

  public boolean isOnTarget() {
    return hasTarget()
        && Math.abs(getTx()) < Constants.LimelightConstants.AIM_TOLERANCE_DEG;
  }

  public boolean isAtDistance() {
    double distance = getDistanceMeters();
    return distance != INVALID_DISTANCE
        && Math.abs(distance - Constants.LimelightConstants.TARGET_DISTANCE_METERS)
            < Constants.LimelightConstants.DISTANCE_TOLERANCE_METERS;
  }

  public boolean isReadyToScore() {
    return isOnTarget() && isAtDistance();
  }

  @Override
  public void periodic() {
    boolean hasTarget = hasTarget();
    double tx = getTx();
    double ty = getTy();
    double ta = getTa();
    double distance = getDistanceMeters();

    SmartDashboard.putBoolean("Limelight/Has Target", hasTarget);
    SmartDashboard.putNumber("Limelight/TX", tx);
    SmartDashboard.putNumber("Limelight/TY", ty);
    SmartDashboard.putNumber("Limelight/TA", ta);

    SmartDashboard.putNumber("Limelight/Distance Meters", distance);
    SmartDashboard.putBoolean("Limelight/Distance Valid", distance != INVALID_DISTANCE);

    SmartDashboard.putBoolean("Limelight/On Target", isOnTarget());
    SmartDashboard.putBoolean("Limelight/At Distance", isAtDistance());
    SmartDashboard.putBoolean("Limelight/Ready To Score", isReadyToScore());

    SmartDashboard.putNumber("Limelight/Distance Error", distanceErrorMeters);
    SmartDashboard.putNumber("Limelight/Approach Forward", approachForward);
    SmartDashboard.putNumber("Limelight/Approach Turn", approachTurn);

    SmartDashboard.putNumber("Limelight/Vision State", visionState);
    SmartDashboard.putBoolean("Limelight/Idle", isIdle());
    SmartDashboard.putBoolean("Limelight/Seeking", isSeeking());
    SmartDashboard.putBoolean("Limelight/Aiming", isAiming());
    SmartDashboard.putBoolean("Limelight/Driving", isDriving());
    SmartDashboard.putBoolean("Limelight/Ready", isReady());

    SmartDashboard.putString(
        "Limelight/Debug",
        "tv=" + hasTarget
            + " tx=" + tx
            + " ty=" + ty
            + " dist=" + distance);
  }
}