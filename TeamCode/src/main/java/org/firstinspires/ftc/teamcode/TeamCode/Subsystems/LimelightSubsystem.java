package org.firstinspires.ftc.teamcode.TeamCode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.TeamCode.Globals;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    Limelight3A limelight;
    IMU imu;
    LLResult result;
    double tx;
    double ty;

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    double ta;
    public enum LimelightMode {
        READ_PATTERN,
        TRACK_ARTIFACT,
        BASKET,
        PAUSE
    }

    public LimelightSubsystem(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        imu = hwMap.get(IMU.class, "imu");
        limelight.setPollRateHz(10);
        limelight.start();
    }

    public void setMode(LimelightMode mode) {
        switch (mode) {
            case READ_PATTERN:
                limelight.pipelineSwitch(0);
                break;
            case TRACK_ARTIFACT:
                limelight.pipelineSwitch(1);
                break;
            case BASKET:
                if (Globals.team_color == Globals.TEAM.RED) {
                    limelight.pipelineSwitch(2);
                } else {
                    limelight.pipelineSwitch(3);
                }
                break;
            case PAUSE:
                limelight.pause();
                break;
        }
    }

    public void getBasicResults() {
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image);
        }
    }

    public void MegaTag2D() {
        // First, tell Limelight which way your robot is facing
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }
    }

    public void AprilTagPose() {
        List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult aprilTag : aprilTags) {
            int id = aprilTag.getFiducialId(); // The ID number of the aprilTag
            double x = aprilTag.getTargetXDegrees(); // Where it is (left-right)
            double y = aprilTag.getTargetYDegrees(); // Where it is (up-down)
            double StrafeDistance_3D = aprilTag.getRobotPoseFieldSpace().getPosition().y;
        }
    }

    public void periodic() {
        result = limelight.getLatestResult();
    }
}