// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetClawState;
import frc.robot.commands.SetIntakeDeployState;
import frc.robot.commands.SetIntakeSpeed;
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
public class AutoGroundIntakeCone extends ParallelCommandGroup {

  /** Creates a new AutoGroundIntakeCube. */
  public AutoGroundIntakeCone(Elevator elevator, Arm arm, Claw claw, Intake intake,
      IntakeDeploy intakeDeploy, Spindexer spindexer) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SafeDumbTowerToPosition(elevator, arm, Constants.TowerConstants.intakeBackstop).asProxy(),
        new SetClawState(claw, ClawState.Closed),
        new SetIntakeDeployState(intakeDeploy, IntakeDeployState.GroundIntake),
        new SetIntakeSpeed(intake, 1.0, .7),
        new AutoSpinSpindexer(spindexer).repeatedly());
  }
}
