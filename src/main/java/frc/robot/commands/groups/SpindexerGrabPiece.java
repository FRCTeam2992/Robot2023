// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetClawState;
import frc.robot.commands.SetIntakeDeployState;
import frc.robot.commands.StopSpindexer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpindexerGrabPiece extends SequentialCommandGroup {
    /** Creates a new FinishIntakeSequence. */

    RobotState robotState;
    Elevator elevator;
    Arm arm;

    public SpindexerGrabPiece(Elevator elevator, Arm arm, Claw claw, Intake intake, IntakeDeploy intakeDeploy,
            Spindexer spindexer, RobotState robotState) {
        // Add your commands in the addCommands() call, e.g.
        this.robotState = robotState;
        this.elevator = elevator;
        this.arm = arm;

        addCommands(
                new SetClawState(claw, ClawState.Opened),
                new ParallelRaceGroup(
                        new MoveIntake(intake, 0, 0),
                        new SetIntakeDeployState(intakeDeploy, IntakeDeployState.Normal),
                        // new MoveSpindexer(spindexer,
                        // -Constants.SpindexerConstants.AutoSpin.motorSpeed),
                        new StopSpindexer(spindexer),
                        moveIntakeToGrabPosition(elevator, arm, robotState).asProxy()),
                // new AutoSpinSpindexer(s).repeatedly().withTimeout(1.5)),
                new SetClawState(claw, ClawState.Closed));
    }

    private SelectCommand moveIntakeToGrabPosition(Elevator elevator, Arm arm, RobotState robotState) {
        return new SelectCommand(
                Map.ofEntries(
                        Map.entry(
                                RobotState.IntakeModeState.Cone,
                                                        new SafeDumbTowerToPosition(elevator, arm,
                                                                        Constants.TowerConstants.intakeGrabCone)
                                                                        .andThen(new SetArmPosition(arm, -2.0))),
                        Map.entry(
                                RobotState.IntakeModeState.Cube,
                                new SafeDumbTowerToPosition(elevator, arm, Constants.TowerConstants.intakeGrabCube)),
                        Map.entry(
                                RobotState.IntakeModeState.Unknown,
                                                        new SafeDumbTowerToPosition(elevator, arm,
                                                                        Constants.TowerConstants.intakeGrabCone)
                                                                        .andThen(new SetArmPosition(arm, -2.0)))),
                robotState::getIntakeMode);

    }

}
