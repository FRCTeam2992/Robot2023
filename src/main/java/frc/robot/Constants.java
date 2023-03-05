// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.manipulator.Waypoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean dataLogging = true;

  public static class DrivetrainConstants {
    // Drive Variables
    public static final boolean isFieldCentric = true;
    public static final boolean isVelocityControlled = true;
    public static final boolean isGyroCorrected = true;
    public static final double joystickDeadband = 0.15;
    public static double joystickXYSmoothFactor = 0.5;
    public static double joystickRotationSmoothFactor = 0.5;
    public static double joystickRotationInverseDeadband = 0.14;

    // Length and Width of the Robot in Meters (Inches: 22.0 x 24.5)
    public static final double swerveWidth = 0.578;
    public static final double swerveLength = 0.667;

    // Max Swerve Speed (Velocity Control)
    public static final double swerveMaxSpeed = 3; // (Meters per Second)(2 Slow, 4.5 normal)

    // Swerve Wheels and Gear Ratio
    public static final double driveGearRatio = 6.75;// 6.75:1
    public static final double driveWheelDiameter = 0.098552;

    // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
    // Right
    public static final double frontLeftOffset = 153.15;
    public static final double frontRightOffset = 90.4;
    public static final double rearLeftOffset = -98.2;
    public static final double rearRightOffset = 6.2;

    public static class PIDConstants {
      // Swerve Drive PID (Velocity Control)
      public static final double driveP = 0.05;// .05
      public static final double driveI = 0.0;// .0
      public static final double driveD = 0.01;
      public static final double driveF = 0.047;

      // Swerve Turn PIDs
      public static final double turnP = 0.013; // .013
      public static final double turnI = 0.0;// .0
      public static final double turnD = 0.00005;
    }

    // Gyro P
    public static final double driveGyroP = 0.005;

    // Swerve Module Translations x=.591/2 y=.654/2
    public static final Translation2d frontLeftLocation = new Translation2d(0.289, 0.3335);
    public static final Translation2d frontRightLocation = new Translation2d(0.289, -0.3335);
    public static final Translation2d rearLeftLocation = new Translation2d(-0.289, 0.3335);
    public static final Translation2d rearRightLocation = new Translation2d(-0.289, -0.3335);

    // Swerve X Axis Correction PID (Path Following)
    public static final double xCorrectionP = 5.0;
    public static final double xCorrectionI = 0.0;
    public static final double xCorrectionD = 0.0;

    // Swerve Y Axis Correction PID (Path Following)
    public static final double yCorrectionP = 5.0;
    public static final double yCorrectionI = 0.0;
    public static final double yCorrectionD = 0.0;

    // Swerve Theta Axis Correction PID (Path Following)
    public static final double thetaCorrectionP = 3.0;
    public static final double thetaCorrectionI = 0.0;
    public static final double thetaCorrectionD = 0.0;

    // Max Path Following Drive Speeds
    public static final double maxPathFollowingVelocity = 3.0; // (Meters per Second)
    public static final double maxPathFollowingAcceleration = 2; // (Meters per Second Squared)

    // Max Path Following Turn Speeds
    public static final double maxThetaVelocity = 6.28; // (Radians per Second)
    public static final double maxThetaAcceleration = 6.28; // (Radians per Second Squared)

    // Max speeds where its safe to X wheels
    public static final double maxSpeedToX = 0.25; // m/sec
    public static final double maxTurnToX = 20.0; // degrees/sec

    public static class CanIDs {
      public static int frontLeftDrive = 2;
      public static int frontLeftTurn = 3;
      public static int frontRightDrive = 4;
      public static int frontRightTurn = 5;
      public static int rearLeftDrive = 6;
      public static int rearLeftTurn = 7;
      public static int rearRightDrive = 8;
      public static int rearRightTurn = 9;

      public static int frontLeftEncoder = 3;
      public static int frontRightEncoder = 5;
      public static int rearLeftEncoder = 7;
      public static int rearRightEncoder = 9;
    }

    // Field Coordinates
    public static class FieldSize {
      public static double FIELD_WIDTH_METERS = 8.02;
      public static double FIELD_LENGTH_METERS = 16.04;
    }
  }

  public static class IntakeConstants {
    public static class DeviceIDs {
      public static int intakeMotorTop = 21;
      public static int intakeMotorBottom = 22;

      public static int intakeSolenoid = 4;
    }
  }

  public static class IntakeDeployConstants {
    public static class DeviceIDs {
      public static int intakeDeployMotor = 23;

      public static int intakeDeployLimitSwitch = 0;
      public static int intakeDeployLimitSwitch2 = 1;

    }

    public static class PIDConstants {
      public static double P = 0.01;
      public static double I = 0;
      public static double D = 0;
      public static double FF = 0;
    }
  }

  public static class SpindexerConstants {
    public static class DeviceIDs {
      public static int spindexerMotor = 24;
    }

    public static class AutoSpin {
      public static double motorSpeed = 0.6; // percent output
      public static double timeoutForDirectionChange = 1.0; // seconds
    }
  }

  public static class TowerConstants {
    public static Waypoint scoreFloor = new Waypoint(0.0, 117.0);
    public static Waypoint scoreConeMid = new Waypoint(0.0, 219.0);
    public static Waypoint scoreConeHigh = new Waypoint(23.5, 199.0);
    public static Waypoint scoreCubeMid = new Waypoint(0.0, 199.0);
    public static Waypoint scoreCubeHigh = new Waypoint(27.25, 166.0);
    public static Waypoint intakeBackstop = new Waypoint(9.75, 35);
    public static Waypoint intakeGrab = new Waypoint(0.0, 0.0);
    public static Waypoint intakeRegrab = new Waypoint(7.75, 5.0);
    public static Waypoint floorGrab = new Waypoint(0.0, 92.0);
  }

  public static class ElevatorConstants {
    public static class DeviceIDs {
      public static int elevatorMotorLead = 25;
      public static int elevatorMotorFollow = 26;

      public static int elevatorSolenoid = 1;
    }

    public static class PIDConstants {
      public static double P = 0.4;
      public static double I = 0;
      public static double D = 0.5;
      public static double FF = 0.1;
      public static double cruiseVelocity = 13200;
      // public static double cruiseVelocity = 1000;
      public static double acceleration = 26400;
      // public static double acceleration = 2000;
    }

    public static class Limits {
      public static double hardStopTop = 32.25;
      public static double hardStopBottom = 0.0;
      public static double softStopTop = 31.75;
      public static double softStopBottom = 0.5;
    }

    public static int encoderClicksPerRevolution = 2048; // clicks per revolution
    public static double gearRatio = 6.0; // 6:1 ratio
    public static double sprocketPitchDiameter = 1.751; // inches
    public static double elevatorHeightToleranceInch = 0.5; // Moves within .5 inch are "close enough

    public static double encoderClicksPerInch = (encoderClicksPerRevolution * gearRatio)
        / (sprocketPitchDiameter * Math.PI);

  }

  public static class ArmConstants {
    public static class DeviceIDs {
      public static int armMotor = 27;
      public static int armEncoder = 27;
    }

    public static class PIDConstants {
      public static double P = 1;
      public static double I = 0;
      public static double D = 0;
      public static double FF = 0;

      public static double cruiseVelocity = 15000;
      public static double acceleration = 29000;
      // public static double cruiseVelocity = 3000;
      // public static double acceleration = 6000;

    }

    public static class Limits {
      public static double hardStopTop = 221.0;
      public static double hardStopBottom = -9.0;
      public static double softStopTop = 216.0;
      public static double softStopBottom = -4.0;
    }

    public static double gearRatio = 128.0;
    public static double motorEncoderClicksPerDegree = (2048.0 * gearRatio) / 360.0;
    public static double armAngleToleranceDeg = 1.0; // Moves within 1 degree are "close enough"

    public static double CANCoderOffset = -67.9;

    public static class ArmSlopConstants {
      // All of these constants need to be validated!
      public static double topZoneEdge = 207; // Higher angle than this assume arm slop, adjust encoder by slop amount
      public static double topZoneAdjustment = 9.6; // True encoder arm position this much lower than Cancoder in this
      // zone
      public static double bottomZoneEdge = 40; // If lower angle than this assume arm slop, adjust encoder by slop
                                                // amount
      public static double bottomZoneAdjustment = 9.6;
    }

  }

  public static class ClawConstants {
    public static class DeviceIDs {
      public static int clawSolenoid = 0;

    }
  }

  public static class ButterflyWheelsConstants {
    public static class DeviceIDs {
      public static int butterflyWheelsSolenoid = 2;
    }
  }
}
