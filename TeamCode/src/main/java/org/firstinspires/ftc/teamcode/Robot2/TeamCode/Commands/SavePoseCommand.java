package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.InstantCommand;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

public class SavePoseCommand extends InstantCommand {
    Follower follower;

    public SavePoseCommand(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
        lastAutoPose = follower.getPose();
    }
}
