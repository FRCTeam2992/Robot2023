// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.leds.Color;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer mRobotContainer;

  private int slowLoopCounter = 0;
  private int ledsLoopCounter = 0;

  // public static AddressableLED m_led;
  // public static AddressableLEDBuffer m_ledBuffer;

  // public static Color purple = new Color(210, 75, 230);
  // public static Color yellow = new Color(255, 160, 0);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    mRobotContainer = new RobotContainer();

    mRobotContainer.mDrivetrain.navx.zeroYaw();

    mRobotContainer.mElevator.zeroElevatorEncoders();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // PWM port 0
    // Must be a PWM header, not MXP or DIO

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    if (slowLoopCounter++ < 5) {
      slowLoopCounter = 0;
      mRobotContainer.addRobotStateToDashboard();
    }

    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    mRobotContainer.mIntake.onDisable();
    mRobotContainer.mElevator.onDisable();
    mRobotContainer.mArm.onDisable();
    mRobotContainer.mClaw.onDisable();
    mRobotContainer.mButterflyWheels.onDisable();
    mRobotContainer.mSpindexer.onDisable();
    mRobotContainer.mIntake.onDisable();
    mRobotContainer.mDrivetrain.onDisable();
  }

  @Override
  public void disabledPeriodic() {
    // Confirm display selected auto status
    SmartDashboard.putString("Confirmed Auto Start Position", mRobotContainer.getAutoStartPosition().description);
    if (mRobotContainer.autoStartCompatible()) {
      SmartDashboard.putString("Confirmed Auto Sequence", mRobotContainer.getAutoSequence().description);
    } else {
      SmartDashboard.putString("Confirmed Auto Sequence", "INVALID SEQUENCE FOR THIS START POSN");
    }
    SmartDashboard.putString("Confirmed Auto Preload Score", mRobotContainer.getAutoPreloadScore().description);

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = mRobotContainer.getAutonomousCommand();

    mRobotContainer.mDrivetrain.setDriveNeutralMode(NeutralMode.Brake);
    mRobotContainer.mDrivetrain.setTurnNeutralMode(NeutralMode.Brake);

    // Set the Drive Motors Current Limit
    mRobotContainer.mDrivetrain.setDriveCurrentLimit(60.0, 60.0);

    // Zero the gyro
    mRobotContainer.mDrivetrain.navx.zeroYaw();

    // Set the Drive Motors Ramp Rate
    mRobotContainer.mDrivetrain.setDriveRampRate(0.0);

    // Arm make sure encoders are current
    mRobotContainer.mArm.initArmMotorEncoder(); // Reset each time we enter Teleop or Auto

    m_autonomousCommand = mRobotContainer.buildAutoCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    mRobotContainer.mDrivetrain.navx.zeroYaw();

    mRobotContainer.mDrivetrain.setDriveNeutralMode(NeutralMode.Brake);
    mRobotContainer.mDrivetrain.setTurnNeutralMode(NeutralMode.Brake);

    mRobotContainer.mDrivetrain.setDriveCurrentLimit(40.0, 40.0);
    mRobotContainer.mDrivetrain.setDriveRampRate(0.25);

    // Arm make sure encoders are current
    mRobotContainer.mArm.initArmMotorEncoder(); // Attempt reset at each teleop init

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (ledsLoopCounter++ < 5) {
    // ledsLoopCounter = 0;
    // if (mRobotContainer.mRobotState.currentColor) {
    // setLEDsColor(yellow);
    // } else {
    // setLEDsColor(purple);
    // }
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

}
