// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ZeroElevatorEncoders extends CommandBase {
 // /** Creates a new ZeroElevatorEncoders. */

  private Elevator mElevator;
  private Timer timer;
  private boolean done = false;
  
  public ZeroElevatorEncoders(Elevator subsystem){
    mElevator = subsystem;

    timer = new Timer();
    addRequirements(mElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mElevator.setElevatorSpeed(0.0);
    timer.reset();
    timer.start();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 0.5) {
      done = true;
      return;
    }
    if (timer.get() > 0.25) {
      mElevator.zeroElevatorEncoders();
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
