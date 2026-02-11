package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;

public class StopperPoseCommand extends InstantCommand {
    Launcher launcher;
    double pose;

    public StopperPoseCommand(Launcher launcher, double pose) {
        this.launcher = launcher;
        this.pose = pose;
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        launcher.setStopperPose(pose);
    }
}

