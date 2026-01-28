package org.firstinspires.ftc.teamcode.Robot2.TeamCode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Globals {
    public enum Alliance {
        RED,
        BLUE
    }

    public static double lltx;
    public static double llty;
    public static double llta;
    public static double llpower;

    public static Alliance alliance = Alliance.BLUE;
    public static Pose lastAutoPose = new Pose(72, 7.5, Math.toRadians(90));

    public static Pose blueGoalPose = new Pose(0, 142, Math.toRadians(0));
    public static Pose redGoalPose = new Pose(142, 142, Math.toRadians(0));
}