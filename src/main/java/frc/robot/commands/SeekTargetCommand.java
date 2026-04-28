package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class SeekTargetCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelightSubsystem;

  private final double maxAngularRate =
      Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond);

  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public SeekTargetCommand(
      CommandSwerveDrivetrain drivetrain,
      LimelightSubsystem limelightSubsystem) {
    this.drivetrain = drivetrain;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    limelightSubsystem.setVisionState(1.0); // seeking
  }

  @Override
  public void execute() {
    if (limelightSubsystem.hasTarget()) {
      stopDrivetrain();
      limelightSubsystem.setVisionState(2.0); // has target / aiming = blink yellow
      return;
    }

    double rotationalRate =
        Constants.LimelightConstants.SEARCH_TURN * maxAngularRate;

    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rotationalRate));

    limelightSubsystem.setVisionState(1.0); // seeking / no target = blink red
  }

  @Override
  public void end(boolean interrupted) {
    stopDrivetrain();
    limelightSubsystem.setVisionState(0.0); // idle = fire
  }

  @Override
  public boolean isFinished() {
    return false; // keep running while left bumper is held
  }

  private void stopDrivetrain() {
    drivetrain.setControl(idleRequest);
  }
}