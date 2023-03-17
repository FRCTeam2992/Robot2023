// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class BalanceRobot extends CommandBase {

    private Drivetrain mDrivetrain;
    private double priorPitch;
    private double currentPitch;
    private double currentPitchDelta;
    private LinearFilter lowPass;
    private int inToleranceCount;
    private boolean recentlyCorrected;
    private int correctionWaitTimer;
    private boolean executeCorrectionNow;
    private int correctionsCompleted;
    private final int WAIT_CYCLES_INTOLERANCE = 100;
    private final int WAIT_CYCLES_NEXT_CORRECTION = 20;

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
        correctionWaitTimer = 0;
        recentlyCorrected = false;
        correctionsCompleted = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        priorPitch = currentPitch;
        currentPitch = lowPass.calculate(mDrivetrain.getRobotPitch());
        currentPitchDelta = currentPitch - priorPitch;
        executeCorrectionNow = !recentlyCorrected || (recentlyCorrected && correctionWaitTimer >= WAIT_CYCLES_NEXT_CORRECTION);
        boolean needForwardCorrection = (mDrivetrain.getRobotPitch() > Constants.DrivetrainConstants.pitchTolerance) &&
                (currentPitchDelta > -Constants.DrivetrainConstants.pitchDeltaTolerance);
        boolean needReverseCorrection = (mDrivetrain.getRobotPitch() < -Constants.DrivetrainConstants.pitchTolerance) &&
                (currentPitchDelta < Constants.DrivetrainConstants.pitchDeltaTolerance);

        if (executeCorrectionNow && needForwardCorrection) {
            mDrivetrain.moveRobotFrontBack(true, Constants.DrivetrainConstants.balanceMoveSpeed * Math.max(0.3, 1 - 0.1 * correctionsCompleted));
            inToleranceCount = 0;
            correctionsCompleted++;
        } else if (executeCorrectionNow && needReverseCorrection) {
            mDrivetrain.moveRobotFrontBack(false, (Constants.DrivetrainConstants.balanceMoveSpeed - .1) * Math.max(0.3, 1 - 0.1 * correctionsCompleted));
            inToleranceCount = 0;
            correctionsCompleted++;
        } else {
            mDrivetrain.stopDrive();
            inToleranceCount++;
            if (recentlyCorrected) {
                correctionWaitTimer++;
            } else {
                recentlyCorrected = true;
                correctionWaitTimer = 0;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return inToleranceCount >= WAIT_CYCLES_INTOLERANCE || (Robot.balanceTimer.get() > 14.0 && DriverStation.isAutonomousEnabled());
    }

}
