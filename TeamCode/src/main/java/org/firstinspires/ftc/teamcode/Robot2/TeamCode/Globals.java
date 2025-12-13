package org.firstinspires.ftc.teamcode.Robot2.TeamCode;

import com.pedropathing.geometry.Pose;

public class Globals {
    public enum TEAM {
        RED,
        BLUE
    }

    public static org.firstinspires.ftc.teamcode.Robot1.TeamCode.Globals.TEAM team_color;
    public static Pose lastAutoPose = new Pose(0, 0, Math.toRadians(90));
}