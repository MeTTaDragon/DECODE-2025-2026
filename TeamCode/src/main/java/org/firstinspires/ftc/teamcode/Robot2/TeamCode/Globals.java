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
    public static double llRx;
    public static double llRy;
    public static double llpower;

    public static double targetVelocity = 0;
    public static double requiredSpeed = 0;
    public static double brakingpower = 0.5;

    public static double imuHeading = 0;

    public static Alliance alliance = Alliance.RED;
    public static Pose lastAutoPose = new Pose(72, 7.5, Math.toRadians(90));

    public static Pose blueGoalPose = new Pose(2, 143, Math.toRadians(0));
    public static Pose redGoalPose = new Pose(143, 143, Math.toRadians(0));
}