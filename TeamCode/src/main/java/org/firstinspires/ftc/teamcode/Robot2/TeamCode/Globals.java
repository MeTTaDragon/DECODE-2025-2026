package org.firstinspires.ftc.teamcode.Robot2.TeamCode;

import com.pedropathing.geometry.Pose;

public class Globals {
    public enum Alliance {
        RED,
        BLUE
    }

    public static double lltx;
    public static double llty;
    public static double llta;
    public static double llpower;

    public static Alliance alliance;
    public static Pose lastAutoPose = new Pose(0, 0, Math.toRadians(90));

    public static Pose blueGoalPose = new Pose(5, 136, Math.toRadians(0));
    public static Pose redGoalPose = new Pose(140, 136, Math.toRadians(0));
}