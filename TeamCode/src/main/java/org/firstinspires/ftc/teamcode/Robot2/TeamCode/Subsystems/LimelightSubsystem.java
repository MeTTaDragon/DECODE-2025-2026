package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    private Limelight3A limelight;
    private IMU imu;

    private LLResult result;

    // Target data
    static double tx;
    static double ty;
    static double ta;

    // Robot Position
    static double robotCoordsX;
    static double robotCoordsY;
    static double robotCoordsZ;
    static int id;

    // --- Simple P-Controller Variables for Alignment ---
    // kP: Turn speed per degree of error.
    // Start small (0.01) and increase if it's too slow. Lower if it oscillates.
    final double kP = 0.03;
    final double MAX_AUTO_TURN = 0.5; // Cap the turning speed

    public enum LimelightMode {
        READ_PATTERN,
        TRACK_ARTIFACT,
        BASKET,
        PAUSE
    }



    private static LimelightMode currentMode;

    public LimelightSubsystem(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");

        // FIX: Initialize the servo to prevent NullPointerException

        // TODO: Uncomment this if you want to use MegaTag2D
        // imu = hwMap.get(IMU.class, "imu");

        limelight.setPollRateHz(10);
    }

    public void init() {
        setMode(LimelightMode.PAUSE);
    }

    public void setMode(LimelightMode mode) {
        currentMode = mode;
        switch (currentMode) {
            case READ_PATTERN:
                limelight.pipelineSwitch(0);
                limelight.start();
                break;
            case TRACK_ARTIFACT:
                limelight.pipelineSwitch(3);
                limelight.start();
                break;
            case BASKET:
                if (alliance == Alliance.RED) {
                    limelight.pipelineSwitch(1);
                    limelight.start();
                } else {
                    limelight.pipelineSwitch(2);
                    limelight.start();
                }
                break;
            case PAUSE:
                limelight.pause();
                break;
        }
    }



    /**
     * Updates the basic target variables (tx, ty, ta) from the latest result.
     */
    public void getBasicResults() {
        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();

            // Update Globals
            lltx = tx;
            llty = ty;
            llta = ta;
        } else {
            // Optional: Reset values if no target is found to prevent ghost data
            lltx = 0;
            llty = 0;
            llta = 0;
        }
    }



    public void MegaTag2D() {
        // Added safety check for IMU
        if (imu == null) return;

        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                robotCoordsX = botpose_mt2.getPosition().x;
                robotCoordsY = botpose_mt2.getPosition().y;
                robotCoordsZ = botpose_mt2.getPosition().z;
            }
        }
    }

    public void AprilTagPose() {
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult aprilTag : aprilTags) {
            id = aprilTag.getFiducialId();
            robotCoordsX = aprilTag.getRobotPoseFieldSpace().getPosition().x;
            robotCoordsY = aprilTag.getRobotPoseFieldSpace().getPosition().y;
            robotCoordsZ = aprilTag.getRobotPoseFieldSpace().getPosition().z;
        }
    }

    // --- Getters ---
    public static double getTx() { return tx; }
    public static double getTy() { return ty; }
    public static double getTa() { return ta; }
    public static double getRobotCoordsZ() { return robotCoordsZ; }
    public static double getRobotCoordsX() { return robotCoordsX; }
    public static double getRobotCoordsY() { return robotCoordsY; }
    public static int getId() { return id; }

    /**
     * Run periodically by the CommandScheduler
     */
    @Override
    public void periodic() {
        // 1. Fetch latest result
        result = limelight.getLatestResult();

        // 2. IMPORTANT: Actually update the variables so getTx() isn't 0
        getBasicResults();

        // 3. Update pose if needed (Optional, only if IMU is active)
        // MegaTag2D();
    }
}