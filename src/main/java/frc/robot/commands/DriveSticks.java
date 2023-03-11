// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain;

public class DriveSticks extends CommandBase {

  // Subsystem Instance
  private Drivetrain mDriveTrain;
  private RobotState mRobotState;

  // Joystick Settings
  private boolean isLeftStrafe = true;

  // Command States
  private double gyroTarget;
  private boolean gyroTargetRecorded;

  private ProfiledPIDController scoreYController;

  public DriveSticks(Drivetrain subsystem, RobotState robotState) {
    // Subsystem Instance
    mDriveTrain = subsystem;
    mRobotState = robotState;

    scoreYController = new ProfiledPIDController(Constants.DrivetrainConstants.AutoScorePIDConstants.scoreP,
            Constants.DrivetrainConstants.AutoScorePIDConstants.scoreI,
            Constants.DrivetrainConstants.AutoScorePIDConstants.scoreD,
            new TrapezoidProfile.Constraints(Constants.DrivetrainConstants.AutoScorePIDConstants.scoreCruise,
                    Constants.DrivetrainConstants.AutoScorePIDConstants.scoreAccel));

    // Set the Subsystem Requirement
    addRequirements(mDriveTrain);
  }

  // Called when the command is initially scheduled./
  @Override
  public void initialize() {
    // Reset the Target Recorded State
    gyroTargetRecorded = false;

    // Get the Joystick Settings
    isLeftStrafe = SmartDashboard.getBoolean("isLeftStrafe", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Joystick Inputs (x1 = Strafe, y1 = Speed, x2 = Rotation)
    double x1;
    double y1;
    double x2;

    if (isLeftStrafe) {
      x1 = -Robot.mRobotContainer.getController0().getLeftX();
      y1 = -Robot.mRobotContainer.getController0().getLeftY();
      x2 = -Robot.mRobotContainer.getController0().getRightX();
    } else {
      x1 = -Robot.mRobotContainer.getController0().getRightX();
      y1 = -Robot.mRobotContainer.getController0().getRightY();
      x2 = -Robot.mRobotContainer.getController0().getLeftX();
    }

    // Get the Joystick Magnitude
    double xyMagnitude = Math.sqrt((x1 * x1) + (y1 * y1));

    // Check the Magnitude Deadband
    if (Math.abs(xyMagnitude) <= Constants.DrivetrainConstants.joystickDeadband) {
      x1 = 0.0;
      y1 = 0.0;
    } else {
      // Get the Polar Angle
      double xyAngle = Math.atan2(y1, x1);

      // Smooth the X and Y Axis
      double smoothedXYMagnitude = (Constants.DrivetrainConstants.joystickXYSmoothFactor * Math.pow(xyMagnitude, 3.0))
          + ((1.0 - Constants.DrivetrainConstants.joystickXYSmoothFactor) * xyMagnitude);

      // Normalize the Magnitude
      smoothedXYMagnitude = Math.max(-1.0, Math.min(1.0, smoothedXYMagnitude));

      // Convert from Polar to X and Y Coordinates
      x1 = smoothedXYMagnitude * Math.cos(xyAngle);
      y1 = smoothedXYMagnitude * Math.sin(xyAngle);
    }

    // Check the Rotation Deadband
    if (Math.abs(x2) <= Constants.DrivetrainConstants.joystickDeadband) {
      x2 = 0.0;
    } else {
      // Smooth the Rotation Axis
      // x2 = (Constants.joystickRotationSmoothFactor * Math.pow(x2, 3.0))
      // + ((1.0 - Constants.joystickRotationSmoothFactor) * x2);

      // Smooth the Rotation Axis and Apply Inverse Deadband
      double tempInverseDeadband = Constants.DrivetrainConstants.joystickRotationInverseDeadband;

      if (mDriveTrain.isInSlowMode()) {
        tempInverseDeadband /= 0.3;
      }
      if (mRobotState.isInEndgameMode()) {
        tempInverseDeadband /= 0.8;
      }

      if (x2 >= 0.0) {
        x2 = tempInverseDeadband
            + (1.0 - tempInverseDeadband)
                * ((Constants.DrivetrainConstants.joystickRotationSmoothFactor * Math.pow(x2, 3.0))
                    + ((1.0 - Constants.DrivetrainConstants.joystickRotationSmoothFactor) * x2));
      } else {
        x2 = -tempInverseDeadband
            + (1.0 - tempInverseDeadband)
                * ((Constants.DrivetrainConstants.joystickRotationSmoothFactor * Math.pow(x2, 3.0))
                    + ((1.0 - Constants.DrivetrainConstants.joystickRotationSmoothFactor) * x2));
      }
    }

    // Lock Rotation to 0 for scoring

    // Check for Movement or autoDrieMode
    if (Math.abs(x1) > 0.0 || Math.abs(y1) > 0.0 || Math.abs(x2) > 0.0 || mDriveTrain.isScoringMode() || mDriveTrain.isLoadingMode()) {

      // Demo Slow Mode
      // x1 /= 4;
      // y1 /= 4;
      // x2 /= 4;

      // Slow the Rotation
      x2 *= (2.0 / 3.0);

      // Check for Slow Mode
      if (mDriveTrain.isInSlowMode()) {
        x1 *= 0.6;
        y1 *= 0.6;
        x2 *= 0.3;
      }
      // Check for Endgame Mode
      if (mRobotState.isInEndgameMode()) {
        x1 *= 0.8;
        y1 *= 0.8;
      }

      // Gyro Input (-180 to 180)
      double gyroValue = mDriveTrain.getGyroYaw();

      if (Math.abs(x1) <= Constants.DrivetrainConstants.joystickDeadband
          && Math.abs(y1) <= Constants.DrivetrainConstants.joystickDeadband) {
        gyroTargetRecorded = false;
      }

      // Gyro Correction
      if (Math.abs(x2) <= Constants.DrivetrainConstants.joystickDeadband
          && Constants.DrivetrainConstants.isGyroCorrected) {

        // Check for Recorded Value
        if (gyroTargetRecorded) {
          // Get the Gyro Value
          double tempGyroValue = gyroValue;

          // Normalize the Target Angle (-180 - 180)
          if (gyroTarget < -180.0) {
            gyroTarget += 360.0;
          } else if (gyroTarget > 180) {
            gyroTarget -= 360.0;
          }

          // Normalize the Gyro Angle (-180 - 180)
          if (tempGyroValue > 180.0) {
            tempGyroValue -= 360;
          } else if (tempGyroValue < -180) {
            tempGyroValue += 360;
          }

          // Get the Gyro Error
          double gyroError = gyroTarget + tempGyroValue;

          // Normalize the Gyro Error (-180 - 180)
          if (gyroError > 180.0) {
            gyroError -= 360.0;
          } else if (gyroError < -180.0) {
            gyroError += 360.0;
          }

          // Calculate Correction Speed
          x2 = gyroError * Constants.DrivetrainConstants.driveGyroP;
        } else {
          // Record a Gyro Value
          gyroTarget = -gyroValue;
          gyroTargetRecorded = true;
        }
      } else {
        // Reset the Target Recorded State
        gyroTargetRecorded = false;
      }
      // Adjust the rotation to align to score
      if (mDriveTrain.isScoringMode()) {
        x2 = mDriveTrain.getGyroYaw();
        if (x2 > 180) {
          x2 -= 360;
        }
        x2 = x2 * Constants.DrivetrainConstants.driveRotationP;

        x2 = Math.min(x2, .40);
        x2 = Math.max(x2, -.40);

        gyroTargetRecorded = false;

        // If inside the right X odometry range try to adjust left/right joystick to aim

        double targetY = mRobotState.currentTargetYCoordinateMeters();
        if ((mDriveTrain.getLatestSwervePose().getX() > Constants.ScoringGridConstants.autoAlignmentAreaMinXMeters) &&
                (mDriveTrain.getLatestSwervePose().getX() < Constants.ScoringGridConstants.autoAlignmentAreaMaxXMeters)
                &&
                (Math.abs(mDriveTrain.getLatestSwervePose().getY()
                        - targetY) < Constants.ScoringGridConstants.autoAlignmentMaxYErrorMeters)) {
            // We are inside the "close enough" range where we can try to autosteer
            x2 = scoreYController.calculate(mDriveTrain.getLatestSwervePose().getY(), targetY);

            // Scale it to our max teleop drive speed
            x2 /= Constants.DrivetrainConstants.swerveMaxSpeed;
        } else {
            // We are too far away so need to reset the profiled PID controller
            scoreYController.reset(mDriveTrain.getLatestSwervePose().getY(),
                    x2 * Constants.DrivetrainConstants.swerveMaxSpeed);
        }
      }
      if (mDriveTrain.isLoadingMode()) {
        double rotationTarget = 90.0;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
          rotationTarget *= -1.0;
        }
        x2 = mDriveTrain.getGyroYaw() - rotationTarget;
        if (x2 > 180) {
          x2 -= 360;
        } else if (x2 < -180) {
          x2 += 360;
        }
        x2 = x2 * Constants.DrivetrainConstants.driveRotationP;

        x2 = Math.min(x2, .40);
        x2 = Math.max(x2, -.40);

        gyroTargetRecorded = false;
      }
      // Calculate the Swerve States
      double[] swerveStates;

      // Check for Field Centric Enabled
      if (Constants.DrivetrainConstants.isFieldCentric && mDriveTrain.getDoFieldOreint()) {
        swerveStates = mDriveTrain.swerveController.calculate(x1, y1, x2, gyroValue);
        // SmartDashboard.putBoolean("Is Field Oriented", true);
      } else {
        swerveStates = mDriveTrain.swerveController.calculate(x1, y1, x2);
        // SmartDashboard.putBoolean("Is Field Oriented", false);

      }

      // Get the Swerve Modules
      SwerveModuleFalconFalcon frontLeft = mDriveTrain.frontLeftModule;
      SwerveModuleFalconFalcon frontRight = mDriveTrain.frontRightModule;
      SwerveModuleFalconFalcon rearLeft = mDriveTrain.rearLeftModule;
      SwerveModuleFalconFalcon rearRight = mDriveTrain.rearRightModule;

      // Command the Swerve Modules
      if (Constants.DrivetrainConstants.isVelocityControlled) {
        frontLeft.setDriveVelocity(swerveStates[0], swerveStates[1]);
        frontRight.setDriveVelocity(swerveStates[2], swerveStates[3]);
        rearLeft.setDriveVelocity(swerveStates[4], swerveStates[5]);
        rearRight.setDriveVelocity(swerveStates[6], swerveStates[7]);
      } else {
        frontLeft.setDrive(swerveStates[0], swerveStates[1]);
        frontRight.setDrive(swerveStates[2], swerveStates[3]);
        rearLeft.setDrive(swerveStates[4], swerveStates[5]);
        rearRight.setDrive(swerveStates[6], swerveStates[7]);
      }
    } else {
      mDriveTrain.stopDrive();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveTrain.stopDrive();
  }
}
