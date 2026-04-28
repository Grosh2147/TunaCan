package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class ApproachTargetCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelightSubsystem;

  private final double maxSpeed =
      TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);

  private final double maxAngularRate =
      Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond);

  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public ApproachTargetCommand(
      CommandSwerveDrivetrain drivetrain,
      LimelightSubsystem limelightSubsystem) {
    this.drivetrain = drivetrain;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    limelightSubsystem.setVisionState(3.0);
    limelightSubsystem.setApproachDebug(0.0, 0.0, 0.0);
  }

  @Override
  public void execute() {
    if (!limelightSubsystem.hasTarget()) {
      stopDrivetrain();
      limelightSubsystem.setVisionState(1.0);
      limelightSubsystem.setApproachDebug(-1.0, 0.0, 0.0);
      return;
    }

    double distanceMeters = limelightSubsystem.getDistanceMeters();

    if (distanceMeters < 0.0) {
      stopDrivetrain();
      limelightSubsystem.setVisionState(1.0);
      limelightSubsystem.setApproachDebug(-1.0, 0.0, 0.0);
      return;
    }

    double tx = limelightSubsystem.getTx();

    double distanceError =
        distanceMeters - Constants.LimelightConstants.TARGET_DISTANCE_METERS;

    double turnCommand = -tx * Constants.LimelightConstants.TURN_kP;

    if (Math.abs(tx) > Constants.LimelightConstants.AIM_TOLERANCE_DEG) {
      turnCommand += -Math.copySign(Constants.LimelightConstants.TURN_MIN, tx);
    }

    turnCommand =
        MathUtil.clamp(
            turnCommand,
            -Constants.LimelightConstants.TURN_MAX,
            Constants.LimelightConstants.TURN_MAX);

    double forwardCommand = 0.0;

    if (Math.abs(tx) < Constants.LimelightConstants.DRIVE_ENABLE_TX_DEG) {
      forwardCommand = distanceError * Constants.LimelightConstants.DRIVE_kP;

      forwardCommand =
          MathUtil.clamp(
              forwardCommand,
              -Constants.LimelightConstants.DRIVE_MAX_REV,
              Constants.LimelightConstants.DRIVE_MAX_FWD);

      if (Math.abs(distanceError)
          < Constants.LimelightConstants.DISTANCE_TOLERANCE_METERS) {
        forwardCommand = 0.0;
      }
    }

    if (Math.abs(tx) < Constants.LimelightConstants.AIM_TOLERANCE_DEG) {
      turnCommand = 0.0;
    }

    double velocityX = forwardCommand * maxSpeed;
    double rotationalRate = turnCommand * maxAngularRate;

    drivetrain.setControl(
        driveRequest
            .withVelocityX(velocityX)
            .withVelocityY(0.0)
            .withRotationalRate(rotationalRate));

    if (limelightSubsystem.isReadyToScore()) {
      limelightSubsystem.setVisionState(4.0);
    } else {
      limelightSubsystem.setVisionState(3.0);
    }

    limelightSubsystem.setApproachDebug(
        distanceError,
        forwardCommand,
        turnCommand);
  }

  @Override
  public void end(boolean interrupted) {
    stopDrivetrain();

    if (limelightSubsystem.isReadyToScore()) {
      limelightSubsystem.setVisionState(4.0);
    } else if (limelightSubsystem.hasTarget()) {
      limelightSubsystem.setVisionState(2.0);
    } else {
      limelightSubsystem.setVisionState(0.0);
    }

    limelightSubsystem.setApproachDebug(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return limelightSubsystem.isReadyToScore();
  }

  private void stopDrivetrain() {
    drivetrain.setControl(idleRequest);
  }
}