package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import java.lang.reflect.Modifier;
import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    private Telemetry telemetry;
    private Limelight3A limelight;
    private IMU imu;
    Follower follower;

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

    Pose llPose;

    // Automatic relocalization rate limiter
    private ElapsedTime relocalizationCooldown = new ElapsedTime();

    public enum LimelightMode {
        READ_PATTERN,
        TRACK_ARTIFACT,
        BASKET,
        PAUSE
    }
    private static LimelightMode currentMode;

    public LimelightSubsystem(HardwareMap hwMap, Follower follower, Telemetry telemetry) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        imu = hwMap.get(IMU.class, "imu");
        this.follower = follower;

        limelight.setPollRateHz(70);

        this.telemetry = telemetry;
    }

    public LimelightSubsystem(HardwareMap hwMap, Follower follower) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        imu = hwMap.get(IMU.class, "imu");
        this.follower = follower;

        limelight.setPollRateHz(70);
    }

    public void init() {
        setMode(LimelightMode.PAUSE);
        relocalizationCooldown.reset();
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
                lltx = 0;
                llty = 0;
                llta = 0;
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
        }
        else{
            lltx = 0;
            llta = 0;
        }
    }

    public void Megatag(){
        if(result != null && result.isValid()){
            Pose3D botpose = result.getBotpose();

            if(botpose != null){
                robotCoordsX = botpose.getPosition().toUnit(DistanceUnit.INCH).x;
                robotCoordsY = botpose.getPosition().toUnit(DistanceUnit.INCH).y;

                llPose = new Pose(robotCoordsX, robotCoordsY, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                llRx = llPose.getX() + 72;
                llRy = llPose.getY() + 72;

                // Automatic relocalization with rate limit
                if(llRx > 0 && llRx < 144 && llRy > 0 && llRy < 144) {
                    if(relocalizationCooldown.seconds() > 2) {
                        follower.setPose(new Pose(llRx, llRy, follower.getHeading()));
                        relocalizationCooldown.reset();
                    }
                }
            }
        }
    }

    public void MegaTag2D() {
        // Added safety check for IMU
        if (follower == null) return;

        double robotYaw = Math.toDegrees(follower.getHeading());
        //double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                robotCoordsX = botpose_mt2.getPosition().toUnit(DistanceUnit.INCH).x;
                robotCoordsY = botpose_mt2.getPosition().toUnit(DistanceUnit.INCH).y;

                llPose = new Pose(robotCoordsX, robotCoordsY, robotYaw, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                llRx = llPose.getX();
                llRy = llPose.getY();

                if(llRx > 0 && llRx < 144 && llRy > 0 && llRy < 144){
                    follower.setPose(new Pose(llRx, llRy, follower.getHeading()));
                }
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
    public LimelightMode getCurrentMode(){
        return currentMode;
    }

    /**
     * Run periodically by the CommandScheduler
     */
    @Override
    public void periodic() {
        if(currentMode == LimelightMode.PAUSE) return;

        // 1. Fetch latest result
        result = limelight.getLatestResult();

        // 2. IMPORTANT: Actually update the variables so getTx() isn't 0
        getBasicResults();

        // 3. Update pose if needed (Optional, only if IMU is active)
        Megatag();

        if(telemetry != null) telemetry.addData("limelight reset pose timer", relocalizationCooldown.seconds());

    }
}