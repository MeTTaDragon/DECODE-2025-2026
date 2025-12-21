package org.firstinspires.ftc.teamcode.Robot2.TeamCode;

import com.pedropathing.geometry.Pose;

public class Globals {
    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance;
    public static Pose lastAutoPose = new Pose(0, 0, Math.toRadians(90));

    public static Pose blueGoalPose = new Pose(12, 136, Math.toRadians(0));
    public static Pose redGoalPose = new Pose(132, 136, Math.toRadians(0));
}