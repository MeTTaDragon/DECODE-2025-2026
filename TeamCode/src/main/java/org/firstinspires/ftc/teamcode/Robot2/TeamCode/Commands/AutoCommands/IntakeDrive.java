package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.CheckLoadCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.ColorSensor;

/**
 * This command drives the robot along a specified path while running the intake.
 * It is a sequential command group that combines path following with intake control.
 */
public class IntakeDrive extends SequentialCommandGroup {
    /**
     * Constructs an IntakeDrive command that follows a path with maxSpeed, runs the intake, waits, and then stops the intake.
     *
     * @param follower The robot's path follower.
     * @param maxSpeed The maximum speed to follow the path.
     * @param path The path chain to follow.
     * @param intake The intake subsystem.
     * @param waitTime The time to wait in milliseconds after reaching the destination before stopping the intake.
     */
    public IntakeDrive(Follower follower, double maxSpeed, PathChain path, Intake intake, ColorSensor colorSensor, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower),
                new WaitCommand(waitTime).raceWith(new CheckLoadCommand(colorSensor)),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE),
                new WaitCommand(20),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }

    /**
     * Constructs an IntakeDrive command that follows a path, runs the intake, waits, and then stops the intake.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param intake The intake subsystem.
     * @param waitTime The time to wait in milliseconds after reaching the destination before stopping the intake.
     */
    public IntakeDrive(Follower follower, PathChain path, Intake intake,ColorSensor colorSensor, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower),
                new WaitCommand(waitTime).raceWith(new CheckLoadCommand(colorSensor)),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE),
                new WaitCommand(20),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }/**
     * Constructs an IntakeDrive command that follows a path, runs the intake, waits, and then stops the intake.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param intake The intake subsystem.
     * @param waitTime The time to wait in milliseconds after reaching the destination before stopping the intake.
     */
    public IntakeDrive(Follower follower, PathChain path, Intake intake, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower),
                new WaitCommand(waitTime),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE),
                new WaitCommand(20),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }/**
     * Constructs an IntakeDrive command that follows a path, runs the intake, waits, and then stops the intake.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param maxSpeed
     * @param intake The intake subsystem.
     * @param waitTime The time to wait in milliseconds after reaching the destination before stopping the intake.
     */
    public IntakeDrive(Follower follower, double maxSpeed, PathChain path, Intake intake, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower),
                new WaitCommand(waitTime),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE),
                new WaitCommand(20),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }


    /**
     * Constructs an IntakeDrive command that follows a path and runs the intake without stopping it upon completion.
     * Use this constructor when you want the intake to continue running after the path is finished.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param intake The intake subsystem.
     */
    public IntakeDrive(Follower follower, PathChain path, Intake intake) {
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower)
        );
    }

    /**
     * Constructs an IntakeDrive command that follows a path with maxSpeed and runs the intake without stopping it upon completion.
     * Use this constructor when you want the intake to continue running after the path is finished.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param intake The intake subsystem.
     */
    public IntakeDrive(Follower follower, double maxSpeed, PathChain path, Intake intake) {
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower)
        );
    }
}
