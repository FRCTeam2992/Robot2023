// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.io.Console;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
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
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroundIntakeCube extends ParallelCommandGroup {

  Elevator mElevator;
  Arm mArm;
  Claw mClaw;
  Intake mIntake;
  IntakeDeploy mIntakeDeploy;

  /** Creates a new AutoGroundIntakeCube. */
  public AutoGroundIntakeCube(Elevator e, Arm a, Claw c, Intake i, IntakeDeploy id) {
    mElevator = e;
    mArm = a;
    mClaw = c;
    mIntake = i;
    mIntakeDeploy = id;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SafeDumbTowerToPosition(mElevator, mArm, Constants.TowerConstants.intakeBackstop),
        new SetClawState(mClaw, ClawState.Closed),
        new SetIntakeDeployState(mIntakeDeploy, IntakeDeployState.GroundIntake),
        new SetIntakeSpeed(mIntake, .75, 0));
  }
}
