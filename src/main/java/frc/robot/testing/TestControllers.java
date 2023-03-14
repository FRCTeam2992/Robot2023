package frc.robot.testing;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.DeployElevator;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveIntakeDeploy;
import frc.robot.commands.RehomeIntakeDeploy;
import frc.robot.commands.SetClawState;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Elevator.ElevatorState;

public class TestControllers {
    private final CommandXboxController testController0;
    private final CommandXboxController testController1;

    public TestControllers() {
        testController0 = new CommandXboxController(2);
        testController1 = new CommandXboxController(3);
    }

    public void configTestButtonBindings(RobotContainer mRobotContainer) {
        /*
         * DO NOT USE "controller0" or "controller1" here
         */
        testController0.povUp().whileTrue(new MoveIntakeDeploy(mRobotContainer.mIntakeDeploy, -0.3));
        testController0.povDown().whileTrue(new MoveIntakeDeploy(mRobotContainer.mIntakeDeploy, 0.10));
        testController0.povRight().onTrue(new RehomeIntakeDeploy(mRobotContainer.mIntakeDeploy));

        testController0.a().whileTrue(new SetIntakeSpeed(mRobotContainer.mIntake, 1, 1));
        testController0.b().whileTrue(new SetIntakeSpeed(mRobotContainer.mIntake, .75, 0));

        testController1.povUp().whileTrue(new MoveElevator(mRobotContainer.mElevator, .1));
        testController1.povDown().whileTrue(new MoveElevator(mRobotContainer.mElevator, -.1));

        testController1.povLeft().whileTrue(new MoveArm(mRobotContainer.mArm, .1));
        testController1.povRight().whileTrue(new MoveArm(mRobotContainer.mArm, -.1));

        testController1.a().onTrue(new DeployElevator(mRobotContainer.mElevator, ElevatorState.Deployed));
        testController1.b().onTrue(new DeployElevator(mRobotContainer.mElevator, ElevatorState.Undeployed));

        testController1.leftBumper().onTrue(new InstantCommand(() -> {
            mRobotContainer.mDrivetrain.setInSlowMode(true);
        }));
        testController1.leftBumper().onFalse(new InstantCommand(() -> {
            mRobotContainer.mDrivetrain.setInSlowMode(false);
        }));

        testController1.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .3)
                        .onTrue(new SetClawState(mRobotContainer.mClaw, ClawState.Closed));
        testController1.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .3)
                        .onFalse(new SetClawState(mRobotContainer.mClaw, ClawState.Opened));
    }
    
}
