// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DeployElevator;
import frc.robot.commands.DriveSticks;
import frc.robot.commands.MoveSpindexer;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetSwerveAngle;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.StopArm;
import frc.robot.commands.StopElevator;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopSpindexer;
import frc.robot.commands.groups.FollowTrajectoryCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ButterflyWheels;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TestPneumatics;
import frc.robot.subsystems.Elevator.ElevatorState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the
 * robot (including subsystems, commands, and trigger mappings) should
 * be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller0 = new CommandXboxController(0);

  public final Drivetrain mDrivetrain;

  public final Intake mIntake;
  public final Spindexer mSpindexer;

  public final Elevator mElevator;
  public final Arm mArm;
  public final Claw mClaw;

  public final ButterflyWheels mButterflyWheels;

  public final TestPneumatics mTestPneumatics;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mDrivetrain = new Drivetrain();
    mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain));

    mIntake = new Intake();
    mIntake.setDefaultCommand(new StopIntake(mIntake));

    mSpindexer = new Spindexer();
    mSpindexer.setDefaultCommand(new StopSpindexer(mSpindexer));

    mElevator = new Elevator();
    mElevator.setDefaultCommand(new StopElevator(mElevator));

    mArm = new Arm();
    mArm.setDefaultCommand(new StopArm(mArm));

    mClaw = new Claw();

    mButterflyWheels = new ButterflyWheels();

    mTestPneumatics = new TestPneumatics();

    // Add subsystems to the dashboard
    addSubsystemsToDashboard();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
   * subclasses for {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link
   * edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Based off of a boolean in ExampleSubsystem
    // new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

    // Button Example (B, While Held)
    // controller0.b().whileTrue(new ExampleCommand(m_exampleSubsystem));

    // Trigger Example (Left Trigger at 30%, When Pressed)
    // controller0.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .3).onTrue(new ExampleCommand(m_exampleSubsystem));

    controller0.leftBumper().onTrue(new MoveSpindexer(mSpindexer, -1));
    controller0.leftBumper().onFalse(new MoveSpindexer(mSpindexer, 0));

    controller0.rightBumper().onTrue(new MoveSpindexer(mSpindexer, .85));
    controller0.rightBumper().onFalse(new MoveSpindexer(mSpindexer, 0));

    controller0.start().onTrue(new ResetGyro(mDrivetrain));


    // controller0.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .1).onTrue(new MoveSpindexer(mSpindexer, -controller0.getLeftTriggerAxis()));
    // controller0.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .1).onFalse(new MoveSpindexer(mSpindexer, 0.0));

    // controller0.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .1).onTrue(new MoveSpindexer(mSpindexer, controller0.getRightTriggerAxis()));
    // controller0.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .1).onFalse(new MoveSpindexer(mSpindexer, 0.0));



    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // controller0.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // controller0.a().onTrue(new DeployElevator(mElevator, ElevatorState.Undeployed));
    // controller0.b().onTrue(new DeployElevator(mElevator, ElevatorState.Deployed));
    controller0.a().onTrue(new DeployElevator(mElevator, ElevatorState.Undeployed));
    controller0.b().onTrue(new DeployElevator(mElevator, ElevatorState.Deployed));
    controller0.povUp().whileTrue(new MoveElevator(mElevator, 0.1));
    controller0.povCenter().onTrue(new StopElevator(mElevator));
    controller0.povDown().whileTrue(new MoveElevator(mElevator, -0.1));

    SmartDashboard.putData("Scoring", new DeployElevator(mElevator, ElevatorState.Undeployed));
    SmartDashboard.putData("Loading", new DeployElevator(mElevator, ElevatorState.Deployed));
    SmartDashboard.putData("Move Elevator Down", new MoveElevator(mElevator, -0.1));
    SmartDashboard.putData("Stop Elevator", new MoveElevator(mElevator, 0.0));
    SmartDashboard.putData("Move Elevator Up", new MoveElevator(mElevator, 0.1));
  

    SmartDashboard.putData("Spin Intake", new MoveSpindexer(mSpindexer, .5));

    SmartDashboard.putData("Reset Odometry", mDrivetrain.ResetOdometry());
    SmartDashboard.putData("0 Wheels", new SetSwerveAngle(mDrivetrain, 0, 0, 0, 0));

    SmartDashboard.putData("Test Path Planner Path", new FollowTrajectoryCommand(mDrivetrain, mDrivetrain.driveStraight, true));

  }   

  public void addSubsystemsToDashboard() {
    SmartDashboard.putData("Drivetrain", mDrivetrain);
    SmartDashboard.putData("Arm", mArm);
    SmartDashboard.putData("Claw", mClaw);
    SmartDashboard.putData("Elevator", mElevator);
    SmartDashboard.putData("Intake", mIntake);
    SmartDashboard.putData("Spindexer", mSpindexer);
    SmartDashboard.putData("Butterfly Wheels", mButterflyWheels);
    SmartDashboard.putData("Test Pneumatics", mTestPneumatics);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public CommandXboxController getController0() {
    return controller0;
  }
}
