// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;

public class SetScoringTarget extends CommandBase {
  private RobotState mRobotState;
  private CommandXboxController mController;

  enum JoystickPOVToAngle {
    Center(-1),
    Up(0),
    UpRight(45),
    Right(90),
    DownRight(135),
    Down(180),
    DownLeft(225),
    Left(270),
    UpLeft(315);

    public int angle;

    private JoystickPOVToAngle(int angle) {
      this.angle = angle;
    }

    public static JoystickPOVToAngle fromValue(int angle) {
      switch (angle) {
        case -1:
          return JoystickPOVToAngle.Center;
        case 0:
          return JoystickPOVToAngle.Up;
        case 45:
          return JoystickPOVToAngle.UpRight;
        case 90:
          return JoystickPOVToAngle.Right;
        case 135:
          return JoystickPOVToAngle.DownRight;
        case 180:
          return JoystickPOVToAngle.Down;
        case 225:
          return JoystickPOVToAngle.DownLeft;
        case 270:
          return JoystickPOVToAngle.Left;
        case 315:
          return JoystickPOVToAngle.UpLeft;
      }
      return JoystickPOVToAngle.Center;
    }
  }

  /** Creates a new SetScoringTarget. */
  public SetScoringTarget(RobotState robotState, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    mRobotState = robotState;
    mController = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mController.x().getAsBoolean()) {
      mRobotState.currentTargetedGrid = RobotState.TargetingGrid.GridDriverLeft;
    } else if (mController.a().getAsBoolean()) {
      mRobotState.currentTargetedGrid = RobotState.TargetingGrid.GridCenter;
    } else if (mController.b().getAsBoolean()) {
      mRobotState.currentTargetedGrid = RobotState.TargetingGrid.GridDriverRight;
    } else {
      // Default to Driver left grid if no button is registered
      mRobotState.currentTargetedGrid = RobotState.TargetingGrid.GridDriverLeft;
    }

    JoystickPOVToAngle direction = JoystickPOVToAngle.fromValue(mController.getHID().getPOV());
    switch (direction) {
      case UpLeft:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.HighRight;
        break;
      case Up:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.HighCenter;
        break;
      case UpRight:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.HighLeft;
        break;
      case Left:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.MidRight;
        break;
      case Center:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.MidCenter;
        break;
      case Right:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.MidLeft;
        break;
      case DownLeft:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.LowRight;
        break;
      case Down:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.LowCenter;
        break;
      case DownRight:
        mRobotState.currentTargetPosition = RobotState.GridTargetingPosition.LowLeft;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
