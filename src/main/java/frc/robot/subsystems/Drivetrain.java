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
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.lib.drive.swerve.SwerveController;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.lib.vision.LimeLight;
import frc.lib.vision.LimeLight.CoordinateSpace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  // Limelights
  public final LimeLight limeLightCamera11;
  public final LimeLight limeLightCamera12;

  private DataLog mDataLog;

  private StringLogEntry limelight11JsonLog;
  private StringLogEntry limelight12JsonLog;

  private DoubleArrayLogEntry ll11BotposeFieldSpaceLog;
  private DoubleArrayLogEntry ll12BotposeFieldSpaceLog;
  private DoubleArrayLogEntry ll11BotposeBlueLog;
  private DoubleArrayLogEntry ll12BotposeBlueLog;
  private DoubleArrayLogEntry ll11BotposeRedLog;
  private DoubleArrayLogEntry ll12BotposeRedLog;
  private IntegerLogEntry ll11TargetIDLog;
  private IntegerLogEntry ll12TargetIDLog;

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

  // public PathPlannerTrajectory testPath;
  public PathPlannerTrajectory driveStraight;

  // Slowmode
  private boolean inSlowMode = false;

  private int dashboardCounter = 0;

  public Drivetrain() {
    // Motor Inits
    frontLeftDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontLeftDrive, "CanBus2");
    initTalonFX(frontLeftDrive, false);

    frontLeftTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontLeftTurn, "CanBus2");
    initTalonFX(frontLeftTurn, true);

    frontRightDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontRightDrive, "CanBus2");
    initTalonFX(frontRightDrive, false);

    frontRightTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontRightTurn, "CanBus2");
    initTalonFX(frontRightTurn, true);

    rearRightDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearRightDrive, "CanBus2");
    initTalonFX(rearRightDrive, false);

    rearRightTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearRightTurn, "CanBus2");
    initTalonFX(rearRightTurn, true);

    rearLeftDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearLeftDrive, "CanBus2");
    initTalonFX(rearLeftDrive, false);

    rearLeftTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearLeftTurn, "CanBus2");
    initTalonFX(rearLeftTurn, true);

    frontRightEncoder = new CANCoder(Constants.DrivetrainConstants.CanIDs.frontRightEncoder, "CanBus2");
    initCANCoder(frontRightEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);

    frontLeftEncoder = new CANCoder(Constants.DrivetrainConstants.CanIDs.frontLeftEncoder, "CanBus2");
    initCANCoder(frontLeftEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);

    rearRightEncoder = new CANCoder(Constants.DrivetrainConstants.CanIDs.rearRightEncoder, "CanBus2");
    initCANCoder(rearRightEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);

    rearLeftEncoder = new CANCoder(Constants.DrivetrainConstants.CanIDs.rearLeftEncoder, "CanBus2");
    initCANCoder(rearLeftEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);

    setDriveNeutralMode(NeutralMode.Coast);
    setTurnNeutralMode(NeutralMode.Brake);

    setDriveCurrentLimit(40.0, 40.0);
    setTurnCurrentLimit(60.0); // potentially unused

    frontLeftController = new PIDController(Constants.DrivetrainConstants.PIDConstants.turnP,
        Constants.DrivetrainConstants.PIDConstants.turnI,
        Constants.DrivetrainConstants.PIDConstants.turnD);
    frontLeftController.enableContinuousInput(-180.0, 180.0);
    frontLeftController.setTolerance(2.0);

    frontRightController = new PIDController(Constants.DrivetrainConstants.PIDConstants.turnP,
        Constants.DrivetrainConstants.PIDConstants.turnI,
        Constants.DrivetrainConstants.PIDConstants.turnD);
    frontRightController.enableContinuousInput(-180.0, 180.0);
    frontRightController.setTolerance(2.0);

    rearLeftController = new PIDController(Constants.DrivetrainConstants.PIDConstants.turnP,
        Constants.DrivetrainConstants.PIDConstants.turnI,
        Constants.DrivetrainConstants.PIDConstants.turnD);
    rearLeftController.enableContinuousInput(-180.0, 180.0);
    rearLeftController.setTolerance(2.0);

    rearRightController = new PIDController(Constants.DrivetrainConstants.PIDConstants.turnP,
        Constants.DrivetrainConstants.PIDConstants.turnI,
        Constants.DrivetrainConstants.PIDConstants.turnD);
    rearRightController.enableContinuousInput(-180.0, 180.0);
    rearRightController.setTolerance(2.0);

    // Set the Drive PID Controllers
    frontLeftDrive.config_kP(0, Constants.DrivetrainConstants.PIDConstants.driveP);
    frontLeftDrive.config_kI(0, Constants.DrivetrainConstants.PIDConstants.driveI);
    frontLeftDrive.config_kD(0, Constants.DrivetrainConstants.PIDConstants.driveD);
    frontLeftDrive.config_kF(0, Constants.DrivetrainConstants.PIDConstants.driveF);

    frontRightDrive.config_kP(0, Constants.DrivetrainConstants.PIDConstants.driveP);
    frontRightDrive.config_kI(0, Constants.DrivetrainConstants.PIDConstants.driveI);
    frontRightDrive.config_kD(0, Constants.DrivetrainConstants.PIDConstants.driveD);
    frontRightDrive.config_kF(0, Constants.DrivetrainConstants.PIDConstants.driveF);

    rearLeftDrive.config_kP(0, Constants.DrivetrainConstants.PIDConstants.driveP);
    rearLeftDrive.config_kI(0, Constants.DrivetrainConstants.PIDConstants.driveI);
    rearLeftDrive.config_kD(0, Constants.DrivetrainConstants.PIDConstants.driveD);
    rearLeftDrive.config_kF(0, Constants.DrivetrainConstants.PIDConstants.driveF);

    rearRightDrive.config_kP(0, Constants.DrivetrainConstants.PIDConstants.driveP);
    rearRightDrive.config_kI(0, Constants.DrivetrainConstants.PIDConstants.driveI);
    rearRightDrive.config_kD(0, Constants.DrivetrainConstants.PIDConstants.driveD);
    rearRightDrive.config_kF(0, Constants.DrivetrainConstants.PIDConstants.driveF);

    // Swerve Modules
    frontLeftModule = new SwerveModuleFalconFalcon(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
        Constants.DrivetrainConstants.frontLeftOffset, frontLeftController,
        Constants.DrivetrainConstants.driveWheelDiameter,
        Constants.DrivetrainConstants.driveGearRatio,
        Constants.DrivetrainConstants.swerveMaxSpeed);

    frontRightModule = new SwerveModuleFalconFalcon(frontRightDrive, frontRightTurn, frontRightEncoder,
        Constants.DrivetrainConstants.frontRightOffset, frontRightController,
        Constants.DrivetrainConstants.driveWheelDiameter,
        Constants.DrivetrainConstants.driveGearRatio,
        Constants.DrivetrainConstants.swerveMaxSpeed);

    rearLeftModule = new SwerveModuleFalconFalcon(rearLeftDrive, rearLeftTurn, rearLeftEncoder,
        Constants.DrivetrainConstants.rearLeftOffset,
        rearLeftController, Constants.DrivetrainConstants.driveWheelDiameter,
        Constants.DrivetrainConstants.driveGearRatio,
        Constants.DrivetrainConstants.swerveMaxSpeed);

    rearRightModule = new SwerveModuleFalconFalcon(rearRightDrive, rearRightTurn, rearRightEncoder,
        Constants.DrivetrainConstants.rearRightOffset, rearRightController,
        Constants.DrivetrainConstants.driveWheelDiameter,
        Constants.DrivetrainConstants.driveGearRatio,
        Constants.DrivetrainConstants.swerveMaxSpeed);

    // Swerve Controller
    swerveController = new SwerveController(
      Constants.DrivetrainConstants.swerveLength,
      Constants.DrivetrainConstants.swerveWidth
    );

    //Load Motion Paths
    loadMotionPaths();

    // Limelight
    limeLightCamera11 = new LimeLight("limelight-eleven");
    limeLightCamera12 = new LimeLight("limelight-twelve");

    if (Constants.dataLogging) {
      mDataLog = DataLogManager.getLog();

      limelight11JsonLog = new StringLogEntry(mDataLog, "/ll/eleven/json");
      limelight12JsonLog = new StringLogEntry(mDataLog, "/ll/twelve/json");

      ll11BotposeFieldSpaceLog = new DoubleArrayLogEntry(mDataLog, "/ll/eleven/botpose_field");
      ll12BotposeFieldSpaceLog = new DoubleArrayLogEntry(mDataLog, "/ll/twelve/botpose_field");
      ll11BotposeBlueLog = new DoubleArrayLogEntry(mDataLog, "/ll/eleven/botpose_blue");
      ll12BotposeBlueLog = new DoubleArrayLogEntry(mDataLog, "/ll/twelve/botpose_blue");
      ll11BotposeRedLog = new DoubleArrayLogEntry(mDataLog, "/ll/eleven/botpose_red");
      ll12BotposeRedLog = new DoubleArrayLogEntry(mDataLog, "/ll/twelve/botpose_red");
      ll11TargetIDLog = new IntegerLogEntry(mDataLog, "/ll/eleven/target_id");
      ll12TargetIDLog = new IntegerLogEntry(mDataLog, "/ll/twelve/target_id");

    }

    // robot gyro initialization
    navx = new AHRS();

    swerveDriveModulePositions[0] = frontLeftModule.getPosition();
    swerveDriveModulePositions[1] = frontRightModule.getPosition();
    swerveDriveModulePositions[2] = rearLeftModule.getPosition();
    swerveDriveModulePositions[3] = rearRightModule.getPosition();

    // Swerve Drive Kinematics
    swerveDriveKinematics = new SwerveDriveKinematics(
      Constants.DrivetrainConstants.frontLeftLocation,
      Constants.DrivetrainConstants.frontRightLocation,
      Constants.DrivetrainConstants.rearLeftLocation,
      Constants.DrivetrainConstants.rearRightLocation
    );

    // Serve Drive Odometry
    swerveDriveOdometry = new SwerveDriveOdometry(
      swerveDriveKinematics,
      // Rotation2d.fromDegrees(navx.getYaw()),
      Rotation2d.fromDegrees(getGyroYaw()),
      swerveDriveModulePositions,
      new Pose2d(0.0, 0.0, new Rotation2d())
    );
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
      Rotation2d.fromDegrees(-getGyroYaw()),
      swerveDriveModulePositions
    );

    if (Constants.dataLogging) {
      limelight11JsonLog.append(limeLightCamera11.getLimelightJson());
      limelight12JsonLog.append(limeLightCamera12.getLimelightJson());

      ll11BotposeFieldSpaceLog.append(limeLightCamera11.getBotPose(CoordinateSpace.Field));
      ll12BotposeFieldSpaceLog.append(limeLightCamera12.getBotPose(CoordinateSpace.Field));
      ll11BotposeBlueLog.append(limeLightCamera11.getBotPose(CoordinateSpace.Blue));
      ll12BotposeBlueLog.append(limeLightCamera12.getBotPose(CoordinateSpace.Blue));
      ll11BotposeRedLog.append(limeLightCamera11.getBotPose(CoordinateSpace.Red));
      ll12BotposeRedLog.append(limeLightCamera12.getBotPose(CoordinateSpace.Red));
      ll11TargetIDLog.append(limeLightCamera11.getTargetID());
      ll12TargetIDLog.append(limeLightCamera12.getTargetID());
    }

    if (dashboardCounter++ >= 5) {
      SmartDashboard.putNumber("front left encoder", frontLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("front right encoder", frontRightModule.getEncoderAngle());
      SmartDashboard.putNumber("back left encoder", rearLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("back right encoder", rearRightModule.getEncoderAngle());

      SmartDashboard.putNumber("Gyro Yaw (raw deg)", navx.getYaw());
      SmartDashboard.putNumber("Gyro Yaw (adj deg)", getGyroYaw());

      SmartDashboard.putNumber("Odometry Rotation (deg)", latestSwervePose.getRotation().getDegrees());
      SmartDashboard.putNumber("Odometry X (in)", (latestSwervePose.getX() * (100 / 2.54)));
      SmartDashboard.putNumber("Odometry Y (in)", (latestSwervePose.getY() * (100 / 2.54)));
      SmartDashboard.putNumber("Odometry X (m)", latestSwervePose.getX());
      SmartDashboard.putNumber("Odometry Y (m)", latestSwervePose.getY());

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

  public void resetOdometry() {
    swerveDriveOdometry.resetPosition(Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions,
        new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public void resetOdometryToPose(Pose2d initialPose) {
    swerveDriveOdometry.resetPosition(Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions, initialPose);
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

  public Pose2d getLatestSwervePose() {
    return latestSwervePose;
  }

  public void setModuleStates(SwerveModuleState[] states) {
    frontLeftModule.setState(states[0]);
    frontRightModule.setState(states[1]);
    rearLeftModule.setState(states[2]);
    rearRightModule.setState(states[3]);

  }

  public boolean isInSlowMode() {
    return inSlowMode;
  }

  public void setInSlowMode(boolean inSlowMode) {
    this.inSlowMode = inSlowMode;
  }

  public void onDisable() {
    setDriveNeutralMode(NeutralMode.Coast);
    setTurnNeutralMode(NeutralMode.Coast);
    stopDrive();
  }

  private void loadMotionPaths() {
    // testPath = PathPlanner.loadPath("Test Path", new PathConstraints(2, 1.5));
    driveStraight = PathPlanner.loadPath("DriveStraight", new PathConstraints(.5, .5));
  }

  public CommandBase ResetOdometry(){
    return runOnce(() -> {
      resetOdometry();
    });
  }
}
