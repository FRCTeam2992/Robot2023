// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.manipulator.Waypoint;
import frc.robot.commands.groups.SafeDumbTowerToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class TestTowerSafeMove extends CommandBase {
  private Elevator mElevator;
  private Arm mArm;

  /** Creates a new TestTowerSafeMove. */
  public TestTowerSafeMove(Elevator elev, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    mElevator = elev;
    mArm = arm;
    addRequirements(elev, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double height;
    double angle;

    height = SmartDashboard.getNumber("ElevTestMoveHeight", 10);
    angle = SmartDashboard.getNumber("ArmTestMoveAngle", 100);
    System.out.println("TowerSafeMove running to " + height + "," + angle);

    CommandScheduler.getInstance().schedule(new SafeDumbTowerToPosition(mElevator, mArm,
        new Waypoint(height, angle)));

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
