// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BalanceRobot extends CommandBase {

    private Drivetrain mDrivetrain;
    private double priorPitch;
    private double currentPitch;
    private double currentPitchDelta;
    private LinearFilter lowPass;
    private int inToleranceCount;

    /** Creates a new BalanceRobot. */
    public BalanceRobot(Drivetrain driveTrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        mDrivetrain = driveTrain;
        lowPass = LinearFilter.movingAverage(3);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        priorPitch = currentPitch = mDrivetrain.getRobotPitch();
        lowPass.reset();
        inToleranceCount = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        priorPitch = currentPitch;
        currentPitch = lowPass.calculate(mDrivetrain.getRobotPitch());
        currentPitchDelta = currentPitch - priorPitch;

        if ((mDrivetrain.getRobotPitch() > Constants.DrivetrainConstants.pitchTolerance) &&
                (currentPitchDelta > -Constants.DrivetrainConstants.pitchDeltaTolerance)) {
            mDrivetrain.moveRobot(0.0, Constants.DrivetrainConstants.balanceMoveSpeed);
            inToleranceCount = 0;
        } else if ((mDrivetrain.getRobotPitch() < -Constants.DrivetrainConstants.pitchTolerance) &&
                (currentPitchDelta < Constants.DrivetrainConstants.pitchDeltaTolerance)) {
            mDrivetrain.moveRobot(180.0, Constants.DrivetrainConstants.balanceMoveSpeed);
            inToleranceCount = 0;
        } else {
            mDrivetrain.stopDrive();
            inToleranceCount++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return inToleranceCount >= 10;
    }

}
