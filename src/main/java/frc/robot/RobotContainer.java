// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DeployButterflyWheels;
import frc.robot.commands.DeployElevator;
import frc.robot.commands.DriveSticks;
import frc.robot.commands.RehomeIntakeDeploy;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveSpindexer;
import frc.robot.commands.SetSwerveAngle;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveIntakeDeploy;
import frc.robot.commands.SetClawState;
import frc.robot.commands.SetIntakeDeployState;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.StopArm;
import frc.robot.commands.StopElevator;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopIntakeDeploy;
import frc.robot.commands.StopSpindexer;
import frc.robot.commands.TestTowerSafeMove;
import frc.robot.commands.ZeroElevatorEncoders;
import frc.robot.commands.groups.AutoGroundIntakeCone;
import frc.robot.commands.groups.AutoGroundIntakeCube;
import frc.robot.commands.groups.FinishIntakeSequence;
import frc.robot.commands.groups.FollowTrajectoryCommand;
import frc.robot.commands.groups.SafeDumbTowerToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ButterflyWheels;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController controller0 = new CommandXboxController(0);
    private final CommandXboxController controller1 = new CommandXboxController(1);
    private final CommandXboxController testController0 = new CommandXboxController(2);
    private final CommandXboxController testController1 = new CommandXboxController(3);

    public final Drivetrain mDrivetrain;

    public final Intake mIntake;
    public final Spindexer mSpindexer;
    public final IntakeDeploy mIntakeDeploy;

    public final Elevator mElevator;
    public final Arm mArm;
    public final Claw mClaw;

    public final ButterflyWheels mButterflyWheels;

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

        mIntakeDeploy = new IntakeDeploy();
        mIntakeDeploy.setDefaultCommand(new StopIntakeDeploy(mIntakeDeploy));

        mElevator = new Elevator();
        mElevator.setDefaultCommand(new StopElevator(mElevator));

        mArm = new Arm();
        mArm.setDefaultCommand(new StopArm(mArm));

        mClaw = new Claw();

        mButterflyWheels = new ButterflyWheels();

        // Add dashboard things
        addSubsystemsToDashboard();
        // Configure the trigger bindings
        configureShuffleboardBindings();
        configRealButtonBindings();
        configTestButtonBindings();
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

    private void configRealButtonBindings() {
        /*
         * DO NOT PUT TEST BUTTONS IN THIS
         * ONLY REAL BUTTONS FOR COMPETITION
         */

        // -----------------------controller0-----------------------

        // ABXY
        // X-Cone Intake
        // controller0.x().onTrue(null); // Intakedeploy go to gorund spot
        // controller0.x().whileTrue(new MoveIntake(mIntake, 1, 1));// cones

        // A-Cube Intake

        controller0.a().onTrue(new AutoGroundIntakeCube(mElevator, mArm, mClaw, mIntake, mIntakeDeploy, mSpindexer));// cubes
        controller0.b().onTrue(new AutoGroundIntakeCone(mElevator, mArm, mClaw, mIntake, mIntakeDeploy, mSpindexer));// cone
        // B-Retract Intake to Normal Spot(Inside Bumpers)
        // controller0.b().onTrue(null);// Retract intake

        // D-Pad
        controller0.povDown().whileTrue(new SetSwerveAngle(mDrivetrain, 45, -45, -45, 45));// X the wheels
        controller0.povLeft().whileTrue(new SetSwerveAngle(mDrivetrain, 45, -45, -45, 45));// X the wheels
        controller0.povUp().whileTrue(new SetSwerveAngle(mDrivetrain, 45, -45, -45, 45));// X the wheels

        // controller0.povRight().onTrue(null);// HomeIntakeDeploy

        // Bumpers/Triggers
        controller0.leftBumper().onTrue(new InstantCommand(
                () -> {
                    mDrivetrain.setDoFieldOreint(false);
                }));// Disable Field Orient
        controller0.leftBumper().onFalse(new InstantCommand(
                () -> {
                    mDrivetrain.setDoFieldOreint(true);
                }));// Disable Field Orient

        controller0.rightBumper().onTrue(new InstantCommand(
                () -> {
                    mDrivetrain.setInSlowMode(true);
                })); // Slow Mode
        controller0.rightBumper().onFalse(new InstantCommand(
                () -> {
                    mDrivetrain.setInSlowMode(false);
                })); // Slow Mode

        controller0.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .3)
                .onTrue(new SetClawState(mClaw, ClawState.Closed));
        testController1.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .3)
                .onFalse(new SetClawState(mClaw, ClawState.Opened));
        // controller0.axisGreaterThan(XboxController.Axis.kLeftTrigger.value,
        // .60).whileTrue(null);// Auto align for scoring

        // controller0.axisGreaterThan(XboxController.Axis.kRightTrigger.value,
        // .60).onTrue(null);// Toggle claw state

        // Back and Start

        // Joysticks and Buttons

        // -----------------------controller1-----------------------
        // ABXY
        controller1.y().onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.intakeBackstop));

        // Score on hybrid level
        controller1.povDown().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreFloor));
        controller1.povDownLeft().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreFloor));
        controller1.povDownRight().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreFloor));

        // Score mid level
        controller1.povLeft().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreConeMid));
        controller1.povCenter().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreCubeMid));
        controller1.povRight().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreConeMid));

        // Score top level
        controller1.povUpLeft().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreConeHigh));
        controller1.povUp().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreCubeHigh));
        controller1.povUpRight().and(controller1.leftTrigger(0.6)).onTrue(
                new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.scoreConeHigh));

        // Bumper/Trigger
        controller1.leftBumper().whileTrue(new MoveSpindexer(mSpindexer, -0.3));
        controller1.rightBumper().whileTrue(new MoveSpindexer(mSpindexer, 0.3));
        controller1.rightTrigger(0.6).onTrue(
                new FinishIntakeSequence(mElevator, mArm, mClaw, mIntake, mIntakeDeploy, mSpindexer));

        // Back and Start

        // Joysticks and Buttons
        controller1.axisLessThan(XboxController.Axis.kLeftY.value, -0.6).whileTrue(
                new MoveArm(mArm, 0.1));
        controller1.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.6).whileTrue(
                new MoveArm(mArm, -0.1));

        controller1.axisLessThan(XboxController.Axis.kRightY.value, -0.6).whileTrue(
                new MoveElevator(mElevator, 0.1));
        controller1.axisGreaterThan(XboxController.Axis.kRightY.value, 0.6).whileTrue(
                new MoveElevator(mElevator, -0.1));

    }

    private void configTestButtonBindings() {
        /*
         * DO NOT USE "controller0" or "controller1" here
         */
        testController0.povUp().whileTrue(new MoveIntakeDeploy(mIntakeDeploy, -0.3));
        testController0.povDown().whileTrue(new MoveIntakeDeploy(mIntakeDeploy, 0.10));
        testController0.povRight().onTrue(new RehomeIntakeDeploy(mIntakeDeploy));

        testController0.a().whileTrue(new SetIntakeSpeed(mIntake, 1, 1));
        testController0.b().whileTrue(new SetIntakeSpeed(mIntake, .75, 0));

        testController1.povUp().whileTrue(new MoveElevator(mElevator, .1));
        testController1.povDown().whileTrue(new MoveElevator(mElevator, -.1));

        testController1.povLeft().whileTrue(new MoveArm(mArm, .1));
        testController1.povRight().whileTrue(new MoveArm(mArm, -.1));

        testController1.a().onTrue(new DeployElevator(mElevator, ElevatorState.Deployed));
        testController1.b().onTrue(new DeployElevator(mElevator, ElevatorState.Undeployed));

        testController1.leftBumper().onTrue(new InstantCommand(() -> {
            mDrivetrain.setInSlowMode(true);
        }));
        testController1.leftBumper().onFalse(new InstantCommand(() -> {
            mDrivetrain.setInSlowMode(false);
        }));

        testController1.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .3)
                .onTrue(new SetClawState(mClaw, ClawState.Closed));
        testController1.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .3)
                .onFalse(new SetClawState(mClaw, ClawState.Opened));

    }

    private void configureShuffleboardBindings() {
        SmartDashboard.putData("Scoring", new DeployElevator(mElevator, ElevatorState.Undeployed));
        SmartDashboard.putData("Loading", new DeployElevator(mElevator, ElevatorState.Deployed));

        SmartDashboard.putData("Open Claw", new SetClawState(mClaw, ClawState.Opened));
        SmartDashboard.putData("Close Claw", new SetClawState(mClaw, ClawState.Closed));

        SmartDashboard.putData("Move Elevator Down", new MoveElevator(mElevator, -0.1));
        SmartDashboard.putData("Stop Elevator", new MoveElevator(mElevator, 0.0));
        SmartDashboard.putData("Move Elevator Up", new MoveElevator(mElevator, 0.1));
        SmartDashboard.putData("Zero Elevator Encoder", new ZeroElevatorEncoders(mElevator));

        SmartDashboard.putData("Spin Intake", new MoveSpindexer(mSpindexer, .5));

        SmartDashboard.putData("Reset Odometry", mDrivetrain.ResetOdometry());
        SmartDashboard.putData("0 Wheels", new SetSwerveAngle(mDrivetrain, 0, 0, 0, 0));

        SmartDashboard.putData("Home Intake", new RehomeIntakeDeploy(mIntakeDeploy));
        SmartDashboard.putData("Intake to Ground",
                new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.GroundIntake));
        SmartDashboard.putData("Intake to Normal", new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.Normal));
        SmartDashboard.putData("Intake to Load Station",
                new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.LoadStation));
        SmartDashboard.putData("Intake to Home", new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.Homed));

        SmartDashboard.putData("Test Path Planner Path",
                new FollowTrajectoryCommand(mDrivetrain, mDrivetrain.testPath, true));

        SmartDashboard.putData("Deploy Butterfly Wheels", new DeployButterflyWheels(mButterflyWheels));
        SmartDashboard.putData("Test Path Planner Path",
                new FollowTrajectoryCommand(mDrivetrain, mDrivetrain.testPath, true));

        SmartDashboard.putNumber("ElevTestMoveHeight", 20.0);
        SmartDashboard.putNumber("ArmTestMoveAngle", 150);
        SmartDashboard.putData("TestSafeDumbPath", new TestTowerSafeMove(mElevator, mArm));

    }

    public void addSubsystemsToDashboard() {
        SmartDashboard.putData("Drivetrain", mDrivetrain);
        SmartDashboard.putData("Arm", mArm);
        SmartDashboard.putData("Claw", mClaw);
        SmartDashboard.putData("Elevator", mElevator);
        SmartDashboard.putData("Intake", mIntake);
        SmartDashboard.putData("Spindexer", mSpindexer);
        SmartDashboard.putData("Butterfly Wheels", mButterflyWheels);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    // }

    public CommandXboxController getController0() {
        return controller0;
    }
}
