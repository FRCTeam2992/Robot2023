// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.manipulator.Waypoint;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnsafeMoveTowerToPosition extends ParallelCommandGroup {
  /** Creates a new UnsafeMoveTowerToPosition. */

  public UnsafeMoveTowerToPosition(Elevator mElevator, Arm mArm, Waypoint point, boolean hold) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetElevatorPosition(mElevator, point.height(), hold),
        new SetArmPosition(mArm, point.angle(), hold),
        new PrintCommand("UnsafeTowerMove to (" + point.height() + "," + point.angle() + ")"));
  }
}
