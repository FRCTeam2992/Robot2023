// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTrajectoryCommand extends SequentialCommandGroup {
  /** Creates a new FollowTrajectoryCommand. */
  public FollowTrajectoryCommand(Drivetrain mDrivetrain, PathPlannerTrajectory traj, boolean isFirstPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (traj == null) {
        // No trajectory so just end
        addCommands(new InstantCommand());
        return;
    }
    addCommands(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            Pose2d startPose = traj.getInitialHolonomicPose();
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
              startPose = new Pose2d(startPose.getX(),
                  Constants.DrivetrainConstants.FieldSize.FIELD_WIDTH_METERS - startPose.getY(),
                   startPose.getRotation().times(-1));
            }
            mDrivetrain.resetOdometryToPose(startPose);
                    // System.out.println("DEBUG LOG: First path! Pose reset!");
          }
        }),
            // new InstantCommand(() -> {
            // System.out.println("DEBUG LOG: initial holonomic pose = " +
            // traj.getInitialHolonomicPose());
            // System.out.println("DEBUG LOG: initial gyro yaw (adj) = " +
            // mDrivetrain.getGyroYaw());
            // }),
        new PPSwerveControllerCommand(
            traj,
            mDrivetrain.swerveDrivePoseEstimator::getEstimatedPosition, // Pose supplier
            mDrivetrain.swerveDriveKinematics, // SwerveDriveKinematics
            new PIDController(1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
                                        // feedforwards.
            new PIDController(1, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
                                          // only use feedforwards.
            mDrivetrain::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            mDrivetrain // Requires this drive subsystem
            ));

  }
}
