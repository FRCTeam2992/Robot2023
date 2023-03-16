// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.autonomous;

import java.util.HashMap;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.TowerConstants;
import frc.robot.RobotState.GridTargetingPosition;
import frc.robot.RobotState.IntakeModeState;
import frc.robot.commands.BalanceRobot;
import frc.robot.commands.DeployElevator;
import frc.robot.commands.SetClawState;
import frc.robot.commands.SetIntakeDeployState;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.groups.AutoGroundIntakeCube;
import frc.robot.commands.groups.FollowTrajectoryCommand;
import frc.robot.commands.groups.SafeDumbTowerToPosition;
import frc.robot.commands.groups.SpindexerGrabPiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.Spindexer;

/** Add your docs here. */
public class AutoBuilder {
    private RobotState mRobotState;
    private Drivetrain mDrivetrain;
    private Elevator mElevator;
    private Arm mArm;
    private Claw mClaw;
    private Intake mIntake;
    private IntakeDeploy mIntakeDeploy;
    private Spindexer mSpindexer;

    private SendableChooser<AutoStartPosition> autoStartChooser;
    private SendableChooser<AutoSequence> autoSequenceChooser;
    private SendableChooser<AutoPreloadScore> autoPreloadScoreChooser;

    private HashMap<String, Command> eventMap = new HashMap<>();

    public AutoBuilder(RobotState robotState, Drivetrain drivetrain, Elevator elevator,
            Arm arm, Claw claw, Intake intake, IntakeDeploy intakeDeploy, Spindexer spindexer) {
        mRobotState = robotState;
        mDrivetrain = drivetrain;
        mElevator = elevator;
        mArm = arm;
        mClaw = claw;
        mIntake = intake;
        mIntakeDeploy = intakeDeploy;
        mSpindexer = spindexer;

        eventMap.put("AutoGroundIntakeCube", new AutoGroundIntakeCube(mElevator, mArm, mClaw, mIntake,
                mIntakeDeploy, mSpindexer));
        eventMap.put("IntakeDeployGround", new SetIntakeDeployState(intakeDeploy, IntakeDeployState.GroundIntake));
        eventMap.put("IntakeSpeedCone", new SetIntakeSpeed(mIntake, 1.0, 0.85));
        eventMap.put("IntakeSpeedCube", new SetIntakeSpeed(mIntake, 0.75, 0.0));
        eventMap.put("IntakeStop", new SetIntakeSpeed(mIntake, 0.0, 0.0));
        eventMap.put("SetIntakeModeCube", new InstantCommand(() -> mRobotState.intakeMode = IntakeModeState.Cube));
        eventMap.put("SpindexerGrabPiece",
                new SpindexerGrabPiece(elevator, arm, claw, intake, intakeDeploy, spindexer, robotState));
        eventMap.put("TowerMoveBackstop", new SafeDumbTowerToPosition(elevator, arm,
                Constants.TowerConstants.intakeBackstop));
        eventMap.put("IntakeDeployNormal", new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.Normal));
        eventMap.put("TowerMoveHighRight", new SafeDumbTowerToPosition(mElevator, mArm,
                GridTargetingPosition.HighRight.towerWaypoint));
        eventMap.put("DeployElevator", new DeployElevator(mElevator, ElevatorState.Deployed));
        eventMap.put("UndeployElevator", new DeployElevator(mElevator, ElevatorState.Undeployed));
        eventMap.put("IntakeDeployHome", new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.Homed));
        eventMap.put("TowerMoveHighCenter", new SafeDumbTowerToPosition(mElevator, mArm,
                GridTargetingPosition.HighCenter.towerWaypoint));
    }

    public void setupAutoSelector() {
        // Setup choosers for start position
        autoStartChooser = new SendableChooser<>();
        autoStartChooser.addOption(AutoStartPosition.LoadStationEnd.description,
                AutoStartPosition.LoadStationEnd);
        autoStartChooser.addOption(AutoStartPosition.CenterLoadStationSide.description,
                AutoStartPosition.CenterLoadStationSide);
        autoStartChooser.addOption(AutoStartPosition.CenterWallSide.description, AutoStartPosition.CenterWallSide);
        autoStartChooser.setDefaultOption(AutoStartPosition.WallEnd.description,
                AutoStartPosition.WallEnd);

        SmartDashboard.putData("Auto Start Position", autoStartChooser);

        // Setup chooser for preload scoring
        autoPreloadScoreChooser = new SendableChooser<>();
        autoPreloadScoreChooser.addOption(AutoPreloadScore.No_Preload.description, AutoPreloadScore.No_Preload);
        autoPreloadScoreChooser.setDefaultOption(AutoPreloadScore.Hi_Cone.description, AutoPreloadScore.Hi_Cone);

        SmartDashboard.putData("Preload Score?", autoPreloadScoreChooser);

        // Setup chooser for auto sequence
        autoSequenceChooser = new SendableChooser<>();
        autoSequenceChooser.setDefaultOption(AutoSequence.Do_Nothing.description, AutoSequence.Do_Nothing);
        autoSequenceChooser.addOption(AutoSequence.SideMobilityOnly.description, AutoSequence.SideMobilityOnly);
        autoSequenceChooser.addOption(AutoSequence.SideMobilityBalance.description,
                AutoSequence.SideMobilityBalance);
        autoSequenceChooser.addOption(AutoSequence.SideMobilityIntake.description, AutoSequence.SideMobilityIntake);
        autoSequenceChooser.addOption(AutoSequence.Side2Scores.description, AutoSequence.Side2Scores);
        autoSequenceChooser.addOption(AutoSequence.CenterBalance.description, AutoSequence.CenterBalance);

        SmartDashboard.putData("Auto Sequence", autoSequenceChooser);

    }

    public AutoStartPosition getAutoStartPosition() {
        return autoStartChooser.getSelected();
    }

    public AutoPreloadScore getAutoPreloadScore() {
        return autoPreloadScoreChooser.getSelected();
    }

    public AutoSequence getAutoSequence() {
        return autoSequenceChooser.getSelected();
    }

    public boolean autoStartCompatible() {
        // Returns true if the Auto Start Position is valid for the current selected
        // sequence
        return autoSequenceChooser.getSelected().allowedStartPositions.contains(
                autoStartChooser.getSelected());
    }

    private Command setupAutoInitialScoreCommand(PathPlannerTrajectory initialScorePath) {
        Command initialScoreCommand;
        Pose2d startingPose = getAutoStartPosition().getStartPose();
        if (startingPose == null) {
            return new InstantCommand();
        }
        switch (getAutoPreloadScore()) {
            case No_Preload:
                initialScoreCommand = new InstantCommand(() -> mDrivetrain.resetOdometryToPose(startingPose));
                break;
            case Hi_Cone:
                initialScoreCommand = new DeployElevator(mElevator, ElevatorState.Deployed)
                        .andThen(new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.Normal).withTimeout(0.01)
                                .alongWith(new WaitCommand(0.5).andThen(new SafeDumbTowerToPosition(mElevator, mArm,
                                        GridTargetingPosition.HighRight.towerWaypoint)))
                                .alongWith(new WaitCommand(1.7)
                                        .andThen(new FollowTrajectoryCommand(mDrivetrain, initialScorePath, true))));
                // Add Sequential Commands after initial move
                initialScoreCommand = initialScoreCommand
                        .andThen(new WaitCommand(0.2))
                        .andThen(new SetClawState(mClaw, ClawState.Opened))
                        .andThen(new WaitCommand(0.4));
                break;
            default:
                initialScoreCommand = new InstantCommand(() -> mDrivetrain.resetOdometryToPose(startingPose));
        }
        return initialScoreCommand;
    }

    private Command setupAutoPathFollowCommand(boolean isFirstPath) {
        PathPlannerTrajectory path = null;
        Command followCommand = new InstantCommand();
        switch (getAutoSequence()) {
            case Do_Nothing:
                break;
            case SideMobilityOnly:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    path = AutonomousTrajectory.LoadStationMobility.trajectory;
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    path = AutonomousTrajectory.WallMobility.trajectory;
                }
                if (path != null) {
                    followCommand = new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath);
                }
                break;
            case SideMobilityIntake:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    path = AutonomousTrajectory.LoadStationMobilityIntake.trajectory;
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    path = AutonomousTrajectory.WallMobilityIntake.trajectory;
                }
                if (path != null) {
                    followCommand = new FollowPathWithEvents(
                            new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                            path.getMarkers(),
                            eventMap);
                }
                break;
            case Side2Scores:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    path = AutonomousTrajectory.LoadStation2Scores.trajectory;
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    path = AutonomousTrajectory.Wall2Scores.trajectory;
                }
                if (path != null) {
                    followCommand = new FollowPathWithEvents(
                            new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                            path.getMarkers(),
                            eventMap);
                    followCommand = followCommand
                            .andThen(new WaitCommand(0.8).andThen(new SetClawState(mClaw, ClawState.Opened)));
                }
                break;
            case SideMobilityBalance:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationEnd) {
                    path = AutonomousTrajectory.LoadStationMobilityBalance.trajectory;
                } else if (getAutoStartPosition() == AutoStartPosition.WallEnd) {
                    path = AutonomousTrajectory.WallMobilityBalance.trajectory;
                }
                if (path != null) {
                    followCommand = (new FollowPathWithEvents(
                            new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                            path.getMarkers(), eventMap))
                            .andThen(new BalanceRobot(mDrivetrain).andThen(mDrivetrain.XWheels()));
                }
                break;
            case CenterBalance:
                if (getAutoStartPosition() == AutoStartPosition.CenterLoadStationSide) {
                    path = AutonomousTrajectory.CenterBalanceLoadStationSide.trajectory;
                } else if (getAutoStartPosition() == AutoStartPosition.CenterWallSide) {
                    path = AutonomousTrajectory.CenterBalanceWallSide.trajectory;
                }
                if (path != null) {
                    followCommand = new DeployElevator(mElevator, ElevatorState.Undeployed)
                            .andThen(new WaitCommand(2.0))
                            .andThen(new SafeDumbTowerToPosition(mElevator, mArm,
                                    Constants.TowerConstants.intakeBackstop))
                            .andThen(new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath))
                            .andThen(new BalanceRobot(mDrivetrain).andThen(mDrivetrain.XWheels()));
                }
                break;
            default:
        }
        return followCommand;
    }

    public Command buildAutoCommand() {
        PathPlannerTrajectory initialScorepath;
        Command autoPathCommand = null;
        Command initialScoreCommand = null;
        Command afterInitialScoreCommand = null;

        // Ensure Limelight odometry is turned off to prevent
        // overcorrection upon AprilTag sightings during
        // autonomous sequences
        // (This should already be off as it is set in
        // autonomousInit, but this is a failsafe.)
        mRobotState.useLimelightOdometryUpdates = false;

        // Setup the initial preload scoring path and command sequence
        initialScorepath = getAutoStartPosition().getInitialTrajectory();
        initialScoreCommand = setupAutoInitialScoreCommand(initialScorepath);

        if (!autoStartCompatible()) {
            // We have incompatible starting position for sequence.
            // Run only the initial score command, which in the case of
            // No_Preload, just resets odometry and stops.
            return initialScoreCommand;
        } else {
            // Starting position is compatible, so setup the path following command,
            // then build a parallel group to move from scoring position while driving
            if (getAutoPreloadScore() != AutoPreloadScore.No_Preload) {
                autoPathCommand = setupAutoPathFollowCommand(false);
                afterInitialScoreCommand = autoPathCommand;
            } else {
                // In the case of No_Preload, we didn't score, so no arm/elevator/claw
                // reset is needed, and we can just follow the path directly.
                // The path will be our first path, since no initial path is needed if
                // we don't score a preload.
                autoPathCommand = setupAutoPathFollowCommand(true);
                afterInitialScoreCommand = autoPathCommand;
            }

            // If we've completed the above, we should always have a Command object for
            // both initialScoreCommand and afterInitialScoreCommand (either or both of
            // which may be just a dummy InstantCommand that does nothing), so we can now
            // return a sequence of those Commands.
            return initialScoreCommand.andThen(afterInitialScoreCommand);
        }
    }

}
