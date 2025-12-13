package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Globals;

import java.util.List;

/**
 * Manages the Limelight camera for object detection and tracking.
 * This subsystem handles pipeline switching, data fetching, and provides
 * simplified access to target data like tx, ty, and ta.
 */
public class LimelightSubsystem extends SubsystemBase {
    Limelight3A limelight;
    IMU imu;
    LLResult result;
    static double tx;
    static double ty;
    static double ta;
    static double robotCoordsX;
    static double robotCoordsY;
    static double robotCoordsZ;
    static int id;

    Servo llservo = null;



    /**
     * Defines the different operational modes for the Limelight,
     * each corresponding to a specific vision pipeline.
     */
    public enum LimelightMode {
        /** Pipeline for reading the randomization pattern. */
        READ_PATTERN,
        /** Pipeline for tracking the game artifact. */
        TRACK_ARTIFACT,
        /** Pipeline for detecting the basket, switching based on team color. */
        BASKET,
        /** Pauses the Limelight stream to save resources. */
        PAUSE
    }

    public enum LLServoState {
        FRONT,
        BACK
    }

    private static LLServoState currentLLServoState;
    private static LimelightMode currentMode;

    /**
     * Constructs the LimelightSubsystem.
     * Initializes the Limelight camera and IMU from the hardware map and
     * sets the polling rate.
     *
     * @param hwMap The hardware map from the OpMode, used to access physical devices.
     */
    public LimelightSubsystem(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        //imu = hwMap.get(IMU.class, "imu"); //TODO: change to pedro + imu when implemented
        limelight.setPollRateHz(10);
    }


    /**
     * Initializes the Limelight subsystem by setting it to PAUSE mode.
     * This prevents unnecessary processing until a specific mode is selected.
     */
    public void init() {
        setMode(LimelightMode.PAUSE);
        setLLServoState(LLServoState.FRONT);
    }

    /**
     * Sets the Limelight's active pipeline or pauses the stream.
     *
     * @param mode The desired {@link LimelightMode} to switch to.
     */
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
                if (Globals.team_color == Globals.TEAM.RED) {
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

    public void setLLServoState(LLServoState state) {
        currentLLServoState = state;

        switch (currentLLServoState) {
            case FRONT:
                llservo.setPosition(0.0); // Adjust the position value as needed
                break;
            case BACK:
                llservo.setPosition(0.6); // Adjust the position value as needed
                break;
        }
    }
    /**
     * Updates the basic target variables (tx, ty, ta) from the latest result.
     * This method should be called after a new result has been fetched.
     */
    public void getBasicResults() {
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image);
        }
    }

    /**
     * Calculates the robot's field-relative position using MegaTag2D.
     * This requires providing the robot's current yaw from the IMU.
     */
    public void MegaTag2D() {
        // First, tell Limelight which way your robot is facing
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

    /**
     * Example method for processing AprilTag data from the Limelight.
     * Iterates through detected tags and extracts their ID and field-space position.
     */
    // You probably won't need to use these Results Lists. We recommend using the base getTx(), getTy() whenever you can.
    public void AprilTagPose() {
        List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult aprilTag : aprilTags) {
            id = aprilTag.getFiducialId(); // The ID number of the aprilTag
            //double x = aprilTag.getTargetXDegrees(); better to ues classic tx, ty
            //double y = aprilTag.getTargetYDegrees();
            robotCoordsX = aprilTag.getRobotPoseFieldSpace().getPosition().x; //may be usefull
            robotCoordsY = aprilTag.getRobotPoseFieldSpace().getPosition().y;
            robotCoordsZ = aprilTag.getRobotPoseFieldSpace().getPosition().z;
        }
    }

    /**
     * Gets the horizontal offset of the target from the crosshair.
     *
     * @return The target's horizontal angle (tx) in degrees.
     */
    public static double getTx() {
        return tx;
    }

    /**
     * Gets the vertical offset of the target from the crosshair.
     *
     * @return The target's vertical angle (ty) in degrees.
     */
    public static double getTy() {
        return ty;
    }

    /**
     * Gets the area of the target as a percentage of the screen.
     *
     * @return The target area (ta) from 0% to 100%.
     */
    public static double getTa() {
        return ta;
    }

    /**
     * Gets the robot's calculated Z-coordinate in the field space.
     * This value is updated by {@link #MegaTag2D()} or {@link #AprilTagPose()}.
     *
     * @return The robot's Z position, typically representing height.
     */
    public static double getRobotCoordsZ() {
        return robotCoordsZ;
    }

    /**
     * Gets the robot's calculated X-coordinate in the field space.
     * This value is updated by {@link #MegaTag2D()} or {@link #AprilTagPose()}.
     *
     * @return The robot's X position on the field.
     */
    public static double getRobotCoordsX() {
        return robotCoordsX;
    }

    /**
     * Gets the robot's calculated Y-coordinate in the field space.
     * This value is updated by {@link #MegaTag2D()} or {@link #AprilTagPose()}.
     *
     * @return The robot's Y position on the field.
     */
    public static double getRobotCoordsY() {
        return robotCoordsY;
    }

    /**
     * Gets the ID of the last detected AprilTag.
     * This value is updated by the {@link #AprilTagPose()} method.
     *
     * @return The integer ID of the AprilTag.
     */
    public static int getId() {
        return id;
    }


    /**
     * Fetches the latest vision result from the Limelight.
     * This method is intended to be called repeatedly in the main robot loop.
     */
    @Override
    public void periodic() {
        result = limelight.getLatestResult();
    }
}
