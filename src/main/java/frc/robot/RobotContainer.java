// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.leds.Color;
import frc.robot.Constants.TowerConstants;
import frc.robot.commands.DeployButterflyWheels;
import frc.robot.commands.DeployElevator;
import frc.robot.commands.DriveSticks;
import frc.robot.commands.HoldArm;
import frc.robot.commands.HoldElevator;
import frc.robot.commands.RehomeIntakeDeploy;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveSpindexer;
import frc.robot.commands.MoveTowerToScoringPosition;
import frc.robot.commands.SetSwerveAngle;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveIntakeDeploy;
import frc.robot.commands.SetClawState;
import frc.robot.commands.SetIntakeDeployState;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.SetLEDsColor;
import frc.robot.commands.SetScoringTarget;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopIntakeDeploy;
import frc.robot.commands.StopSpindexer;
import frc.robot.commands.TestTowerSafeMove;
import frc.robot.commands.ToggleClawState;
import frc.robot.commands.ToggleDeployElevator;
import frc.robot.commands.ToggleEndgameState;
import frc.robot.commands.ZeroElevatorEncoders;
import frc.robot.commands.groups.AutoGroundIntakeCone;
import frc.robot.commands.groups.AutoGroundIntakeCube;
import frc.robot.commands.groups.AutoLoadStationIntake;
import frc.robot.commands.groups.AutoSpinSpindexer;
import frc.robot.commands.groups.SpindexerGrabPiece;
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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
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

        public final RobotState mRobotState;

        public final Drivetrain mDrivetrain;

        public final Intake mIntake;
        public final Spindexer mSpindexer;
        public final IntakeDeploy mIntakeDeploy;

        public final Elevator mElevator;
        public final Arm mArm;
        public final Claw mClaw;

        public final ButterflyWheels mButterflyWheels;

        private Color purple;
        private Color yellow;

        public AddressableLED m_led;
        public AddressableLEDBuffer m_ledBuffer;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                mRobotState = new RobotState();

                mDrivetrain = new Drivetrain();
                mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain, mRobotState));

                mIntake = new Intake();
                mIntake.setDefaultCommand(new StopIntake(mIntake));

                mSpindexer = new Spindexer();
                mSpindexer.setDefaultCommand(new StopSpindexer(mSpindexer));

                mIntakeDeploy = new IntakeDeploy();
                mIntakeDeploy.setDefaultCommand(new StopIntakeDeploy(mIntakeDeploy));

                mElevator = new Elevator();
                mElevator.setDefaultCommand(new HoldElevator(mElevator));

                mArm = new Arm();
                // mArm.setDefaultCommand(new StopArm(mArm));
                mArm.setDefaultCommand(new HoldArm(mArm));

                mClaw = new Claw();

                mButterflyWheels = new ButterflyWheels();

                m_led = new AddressableLED(0);

                // Reuse buffer
                // Default to a length of 60, start empty output
                // Length is expensive to set, so only set it once, then just update data
                m_ledBuffer = new AddressableLEDBuffer(17);
                m_led.setLength(m_ledBuffer.getLength());

                // Set the data
                m_led.setData(m_ledBuffer);
                m_led.start();

                purple = new Color(210, 75, 230);
                yellow = new Color(255, 160, 0);

                // Add dashboard things
                addSubsystemsToDashboard();

                addRobotStateToDashboard();

                // Configure the trigger bindings
                configureShuffleboardBindings();
                configRealButtonBindings();
                configTestButtonBindings();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
         * constructor with an arbitrary predicate, or via the named factories in
         * t * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
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
        
                controller0.a().onTrue(new InstantCommand(() -> {
                        mDrivetrain.setScoringMode(true);
                }));
                controller0.a().onFalse(new InstantCommand(() -> {
                        mDrivetrain.setScoringMode(false);
                }));
                controller0.b().onTrue(
                        new AutoLoadStationIntake(mElevator, mArm, mClaw, mIntake, mIntakeDeploy, mSpindexer));
                controller0.b().onTrue(new InstantCommand(() -> {
                        mDrivetrain.setLoadingMode(true);
                }));
                controller0.b().onFalse(new InstantCommand(() -> {
                        mDrivetrain.setLoadingMode(false);
                }));
                controller0.x().onTrue(
                                new AutoGroundIntakeCube(mElevator, mArm, mClaw, mIntake, mIntakeDeploy, mSpindexer));// cubes
                controller0.y().onTrue(
                                new AutoGroundIntakeCone(mElevator, mArm, mClaw, mIntake, mIntakeDeploy, mSpindexer));// cone
                // D-Pad
                controller0.povLeft().whileTrue(new SetSwerveAngle(mDrivetrain, 45, -45, -45, 45));// X the wheels

                controller0.povRight().onTrue(new RehomeIntakeDeploy(mIntakeDeploy));

                controller0.povUp().onTrue(new SetLEDsColor(yellow));
                controller0.povDown().onTrue(new SetLEDsColor(purple));

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
                                .onTrue(new ToggleClawState(mClaw));
                controller0.leftTrigger(0.6)
                                .whileTrue(new MoveTowerToScoringPosition(mElevator, mArm, mRobotState));
                controller0.leftTrigger(0.6)
                                .onTrue(new SetIntakeDeployState(mIntakeDeploy, IntakeDeploy.IntakeDeployState.Homed));
                controller0.leftTrigger(0.6).onTrue(new DeployElevator(mElevator, ElevatorState.Deployed));
                controller0.leftTrigger(0.6)
                                .onFalse(new SafeDumbTowerToPosition(mElevator, mArm, TowerConstants.intakeBackstop));
                controller0.leftTrigger(0.6).onFalse(new DeployElevator(mElevator, ElevatorState.Undeployed));

                // Back and Start

                controller0.start().onTrue(new ResetGyro(mDrivetrain));

                // Joysticks Buttons
                controller0.rightStick().onTrue(new MoveIntake(mIntake, -.5, -.5).withTimeout(2));
                controller0.rightStick().onTrue(new AutoSpinSpindexer(mSpindexer).repeatedly());
                controller0.rightStick().onTrue(new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.Normal));

                // -----------------------controller1-----------------------
                // ABXY
                controller1.y().whileTrue(new MoveIntake(mIntake, .5, -.5));

                // Bumper/Trigger
                controller1.leftBumper().whileTrue(new MoveSpindexer(mSpindexer, -0.9));
                controller1.rightBumper().whileTrue(new MoveSpindexer(mSpindexer, 0.9));
                controller1.leftTrigger(0.6).onTrue(
                                new SpindexerGrabPiece(mElevator, mArm, mClaw, mIntake, mIntakeDeploy, mSpindexer));
                controller1.rightTrigger(0.6).onTrue(new SetScoringTarget(mRobotState, controller1));

                // Back and Start
                controller1.start().onTrue(new ToggleEndgameState(mRobotState));
                controller1.back().onTrue(new DeployButterflyWheels(mButterflyWheels)
                                .unless(() -> !mRobotState.isInEndgameMode()));

                // Joysticks and Buttons
                controller1.axisLessThan(XboxController.Axis.kLeftY.value, -0.6).whileTrue(
                                new MoveArm(mArm, 0.20));
                controller1.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.6).whileTrue(
                                new MoveArm(mArm, -0.20));

                controller1.axisLessThan(XboxController.Axis.kRightY.value, -0.6).whileTrue(
                                new MoveElevator(mElevator, 0.2));
                controller1.axisGreaterThan(XboxController.Axis.kRightY.value, 0.6).whileTrue(
                                new MoveElevator(mElevator, -0.2));
                controller1.rightStick().onTrue(new ToggleDeployElevator(mElevator));

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
                SmartDashboard.putData("Intake to Normal",
                                new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.Normal));
                SmartDashboard.putData("Intake to Load Station",
                                new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.LoadStation));
                SmartDashboard.putData("Intake to Home",
                                new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.Homed));

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

        public void addRobotStateToDashboard() {
                SmartDashboard.putBoolean("Target: Left Grid High Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighLeft);
                SmartDashboard.putBoolean("Target: Left Grid High Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighCenter);
                SmartDashboard.putBoolean("Target: Left Grid High Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighRight);
                SmartDashboard.putBoolean("Target: Left Grid Mid Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidLeft);
                SmartDashboard.putBoolean("Target: Left Grid Mid Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidCenter);
                SmartDashboard.putBoolean("Target: Left Grid Mid Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidRight);
                SmartDashboard.putBoolean("Target: Left Grid Low Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowLeft);
                SmartDashboard.putBoolean("Target: Left Grid Low Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowCenter);
                SmartDashboard.putBoolean("Target: Left Grid Low Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverLeft &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowRight);

                SmartDashboard.putBoolean("Target: Center Grid High Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighLeft);
                SmartDashboard.putBoolean("Target: Center Grid High Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighCenter);
                SmartDashboard.putBoolean("Target: Center Grid High Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighRight);
                SmartDashboard.putBoolean("Target: Center Grid Mid Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidLeft);
                SmartDashboard.putBoolean("Target: Center Grid Mid Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidCenter);
                SmartDashboard.putBoolean("Target: Center Grid Mid Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidRight);
                SmartDashboard.putBoolean("Target: Center Grid Low Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowLeft);
                SmartDashboard.putBoolean("Target: Center Grid Low Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowCenter);
                SmartDashboard.putBoolean("Target: Center Grid Low Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridCenter &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowRight);

                SmartDashboard.putBoolean("Target: Right Grid High Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighLeft);
                SmartDashboard.putBoolean("Target: Right Grid High Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighCenter);
                SmartDashboard.putBoolean("Target: Right Grid High Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.HighRight);
                SmartDashboard.putBoolean("Target: Right Grid Mid Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidLeft);
                SmartDashboard.putBoolean("Target: Right Grid Mid Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidCenter);
                SmartDashboard.putBoolean("Target: Right Grid Mid Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.MidRight);
                SmartDashboard.putBoolean("Target: Right Grid Low Left",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowLeft);
                SmartDashboard.putBoolean("Target: Right Grid Low Center",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowCenter);
                SmartDashboard.putBoolean("Target: Right Grid Low Right",
                        mRobotState.currentTargetedGrid == RobotState.TargetingGrid.GridDriverRight &&
                        mRobotState.currentTargetPosition == RobotState.GridTargetingPosition.LowRight);

                SmartDashboard.putBoolean("Blue Alliance",
                        DriverStation.getAlliance() == DriverStation.Alliance.Blue);
                SmartDashboard.putBoolean("Red Alliance",
                        DriverStation.getAlliance() == DriverStation.Alliance.Red);

                SmartDashboard.putBoolean("Endgame Mode",
                        mRobotState.endgameMode == RobotState.EndgameModeState.InEndgame);
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

        public void setLEDsColor(Color color) {
                for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                        // Sets the specified LED to the RGB values for red
                        m_ledBuffer.setRGB(i, color.r(), color.g(), color.b());
                }

                m_led.setData(m_ledBuffer);
        }
}
