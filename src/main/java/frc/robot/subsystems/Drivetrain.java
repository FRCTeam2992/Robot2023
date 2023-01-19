// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import frc.lib.drive.swerve.SwerveController;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  // Motor Contollers
  private TalonFX frontLeftDrive;
  private TalonFX frontLeftTurn;

  private TalonFX frontRightDrive;
  private TalonFX frontRightTurn;

  private TalonFX rearLeftDrive;
  private TalonFX rearLeftTurn;

  private TalonFX rearRightDrive;
  private TalonFX rearRightTurn;

  // Module CAN Encoders
  private final CANCoder frontLeftEncoder;
  private final CANCoder frontRightEncoder;
  private final CANCoder rearLeftEncoder;
  private final CANCoder rearRightEncoder;

  // Turn PID Controllers
  private final PIDController frontLeftController;
  private final PIDController frontRightController;
  private final PIDController rearLeftController;
  private final PIDController rearRightController;

  // Swerve Modules
  public final SwerveModuleFalconFalcon frontLeftModule;
  public final SwerveModuleFalconFalcon frontRightModule;
  public final SwerveModuleFalconFalcon rearLeftModule;
  public final SwerveModuleFalconFalcon rearRightModule;

  // Swerve Controller
  public final SwerveController swerveController;

  // Robot Gyro
  public AHRS navx;
  public double gyroOffset = 0.0;

  public Pose2d latestSwervePose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

  // Swerve Drive Kinematics
  public final SwerveDriveKinematics swerveDriveKinematics;

  // Swerve Drive Odometry
  public final SwerveDriveOdometry swerveDriveOdometry;
  public SwerveModulePosition[] swerveDriveModulePositions = {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  public Transform2d moved;

  // Slowmode
  private boolean inSlowMode = false;

  private int dashboardCounter = 0;

  public Drivetrain() {
    // Motor Inits
    frontRightDrive = new TalonFX(2);
    initTalonFX(frontRightDrive, false);

    frontRightTurn = new TalonFX(3);
    initTalonFX(frontRightTurn, true);

    frontLeftDrive = new TalonFX(4);
    initTalonFX(frontLeftDrive, false);

    frontLeftTurn = new TalonFX(5);
    initTalonFX(frontLeftTurn, true);

    rearRightDrive = new TalonFX(6);
    initTalonFX(rearRightDrive, false);

    rearRightTurn = new TalonFX(7);
    initTalonFX(rearRightTurn, true);

    rearLeftDrive = new TalonFX(8);
    initTalonFX(rearLeftDrive, false);

    rearLeftTurn = new TalonFX(9);
    initTalonFX(rearLeftTurn, true);

    frontRightEncoder = new CANCoder(3);
    initCANCoder(frontRightEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);

    frontLeftEncoder = new CANCoder(5);
    initCANCoder(frontLeftEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);

    rearRightEncoder = new CANCoder(7);
    initCANCoder(rearRightEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);

    rearLeftEncoder = new CANCoder(9);
    initCANCoder(rearLeftEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);

    setDriveNeutralMode(NeutralMode.Coast);
    setTurnNeutralMode(NeutralMode.Brake);

    setDriveCurrentLimit(40.0, 40.0);
    setTurnCurrentLimit(60.0); // potentially unused

    frontLeftController = new PIDController(Constants.DriveConstants.turnP, Constants.DriveConstants.turnI,
        Constants.DriveConstants.turnD);
    frontLeftController.enableContinuousInput(-180.0, 180.0);
    frontLeftController.setTolerance(2.0);

    frontRightController = new PIDController(Constants.DriveConstants.turnP, Constants.DriveConstants.turnI,
        Constants.DriveConstants.turnD);
    frontRightController.enableContinuousInput(-180.0, 180.0);
    frontRightController.setTolerance(2.0);

    rearLeftController = new PIDController(Constants.DriveConstants.turnP, Constants.DriveConstants.turnI,
        Constants.DriveConstants.turnD);
    rearLeftController.enableContinuousInput(-180.0, 180.0);
    rearLeftController.setTolerance(2.0);

    rearRightController = new PIDController(Constants.DriveConstants.turnP, Constants.DriveConstants.turnI,
        Constants.DriveConstants.turnD);
    rearRightController.enableContinuousInput(-180.0, 180.0);
    rearRightController.setTolerance(2.0);

    // Set the Drive PID Controllers
    frontLeftDrive.config_kP(0, Constants.DriveConstants.driveP);
    frontLeftDrive.config_kI(0, Constants.DriveConstants.driveI);
    frontLeftDrive.config_kD(0, Constants.DriveConstants.driveD);
    frontLeftDrive.config_kF(0, Constants.DriveConstants.driveF);

    frontRightDrive.config_kP(0, Constants.DriveConstants.driveP);
    frontRightDrive.config_kI(0, Constants.DriveConstants.driveI);
    frontRightDrive.config_kD(0, Constants.DriveConstants.driveD);
    frontRightDrive.config_kF(0, Constants.DriveConstants.driveF);

    rearLeftDrive.config_kP(0, Constants.DriveConstants.driveP);
    rearLeftDrive.config_kI(0, Constants.DriveConstants.driveI);
    rearLeftDrive.config_kD(0, Constants.DriveConstants.driveD);
    rearLeftDrive.config_kF(0, Constants.DriveConstants.driveF);

    rearRightDrive.config_kP(0, Constants.DriveConstants.driveP);
    rearRightDrive.config_kI(0, Constants.DriveConstants.driveI);
    rearRightDrive.config_kD(0, Constants.DriveConstants.driveD);
    rearRightDrive.config_kF(0, Constants.DriveConstants.driveF);

    // Swerve Modules
    frontLeftModule = new SwerveModuleFalconFalcon(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
        Constants.DriveConstants.frontLeftOffset, frontLeftController, Constants.DriveConstants.driveWheelDiameter,
        Constants.DriveConstants.driveGearRatio,
        Constants.DriveConstants.swerveMaxSpeed);

    frontRightModule = new SwerveModuleFalconFalcon(frontRightDrive, frontRightTurn, frontRightEncoder,
        Constants.DriveConstants.frontRightOffset, frontRightController, Constants.DriveConstants.driveWheelDiameter,
        Constants.DriveConstants.driveGearRatio,
        Constants.DriveConstants.swerveMaxSpeed);

    rearLeftModule = new SwerveModuleFalconFalcon(rearLeftDrive, rearLeftTurn, rearLeftEncoder,
        Constants.DriveConstants.rearLeftOffset,
        rearLeftController, Constants.DriveConstants.driveWheelDiameter, Constants.DriveConstants.driveGearRatio,
        Constants.DriveConstants.swerveMaxSpeed);

    rearRightModule = new SwerveModuleFalconFalcon(rearRightDrive, rearRightTurn, rearRightEncoder,
        Constants.DriveConstants.rearRightOffset, rearRightController, Constants.DriveConstants.driveWheelDiameter,
        Constants.DriveConstants.driveGearRatio,
        Constants.DriveConstants.swerveMaxSpeed);

    // Swerve Controller
    swerveController = new SwerveController(Constants.DriveConstants.swerveLength,
        Constants.DriveConstants.swerveWidth);

    // robot gyro initialization
    navx = new AHRS();

    swerveDriveModulePositions[0] = frontLeftModule.getPosition();
    swerveDriveModulePositions[1] = frontRightModule.getPosition();
    swerveDriveModulePositions[2] = rearLeftModule.getPosition();
    swerveDriveModulePositions[3] = rearRightModule.getPosition();

    // Swerve Drive Kinematics
    swerveDriveKinematics = new SwerveDriveKinematics(Constants.DriveConstants.frontLeftLocation,
        Constants.DriveConstants.frontRightLocation,
        Constants.DriveConstants.rearLeftLocation, Constants.DriveConstants.rearRightLocation);

    // Serve Drive Odometry
    swerveDriveOdometry = new SwerveDriveOdometry(
        swerveDriveKinematics, Rotation2d.fromDegrees(navx.getYaw()),
        swerveDriveModulePositions,
        new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  private void initTalonFX(TalonFX motorContollerName, boolean isInverted) {
    motorContollerName.setInverted(isInverted);
  }

  private void initCANCoder(CANCoder CANCoderName, AbsoluteSensorRange sensorRange, boolean sensorDirection) {
    CANCoderName.configAbsoluteSensorRange(sensorRange);
    CANCoderName.configSensorDirection(sensorDirection);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    swerveDriveModulePositions[0] = frontLeftModule.getPosition();
    swerveDriveModulePositions[1] = frontRightModule.getPosition();
    swerveDriveModulePositions[2] = rearLeftModule.getPosition();
    swerveDriveModulePositions[3] = rearRightModule.getPosition();

    latestSwervePose = swerveDriveOdometry.update(
        Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions);

    if (dashboardCounter++ >= 5) {
      SmartDashboard.putNumber("front left encoder", frontLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("front right encoder", frontRightModule.getEncoderAngle());
      SmartDashboard.putNumber("back left encoder", rearLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("back right encoder", rearRightModule.getEncoderAngle());

      SmartDashboard.putNumber("gyro y", navx.getYaw());
      SmartDashboard.putNumber("gyro y", getGyroYaw());

      SmartDashboard.putNumber("x odometry", latestSwervePose.getX());
      SmartDashboard.putNumber("y odometry", latestSwervePose.getY());

      dashboardCounter = 0;
    }
  }

  public void setDriveNeutralMode(NeutralMode mode) {
    frontLeftDrive.setNeutralMode(mode);
    frontRightDrive.setNeutralMode(mode);
    rearLeftDrive.setNeutralMode(mode);
    rearRightDrive.setNeutralMode(mode);
  }

  public void setTurnNeutralMode(NeutralMode mode) {
    frontLeftTurn.setNeutralMode(mode);
    frontRightTurn.setNeutralMode(mode);
    rearLeftTurn.setNeutralMode(mode);
    rearRightTurn.setNeutralMode(mode);
  }

  public void setDriveCurrentLimit(double currentLimit, double triggerCurrent) {
    frontLeftDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
    frontRightDrive
        .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
    rearLeftDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
    rearRightDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
  }

  // seconds from idle to max speed
  public void setDriveRampRate(double seconds) {
    // Open loop ramp rates
    frontLeftDrive.configOpenloopRamp(seconds);
    frontRightDrive.configOpenloopRamp(seconds);
    rearLeftDrive.configOpenloopRamp(seconds);
    rearRightDrive.configOpenloopRamp(seconds);

    // Closed loop ramp rates
    frontLeftDrive.configClosedloopRamp(seconds);
    frontRightDrive.configClosedloopRamp(seconds);
    rearLeftDrive.configClosedloopRamp(seconds);
    rearRightDrive.configClosedloopRamp(seconds);
  }

  public void setTurnCurrentLimit(double current) {
    frontLeftTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    frontRightTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    rearLeftTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    rearRightTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
  }

  public void stopDrive() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  public double getGyroYaw() {
    double angle = navx.getYaw() + gyroOffset;
    while (angle > 360) {
      angle -= 360;
    }
    while (angle < 0) {
      angle += 360;
    }
    return angle;
  }

  public boolean isInSlowMode() {
    return inSlowMode;
  }

  public void setInSlowMode(boolean inSlowMode) {
    this.inSlowMode = inSlowMode;
  }
}
