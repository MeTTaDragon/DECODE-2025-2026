package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.GlobalsFRI.*;

import androidx.core.view.VelocityTrackerCompat;

@Config
public class Turret extends SubsystemBase {

    private final DcMotorEx motorTureta;
    private final PIDFController turretController;
    private final Follower follower;

    public Pose goalPose;
    public Pose2d targetGoalPose;


    Vector2d targetPosition;

    // Variables for logic
    double robotAngle;
    double power;

    //antistrangulare for always lock on
    public static boolean isChoking;
    private long resetStartTime = 0;
    private boolean isResetting = false;
    private static TurretState stateBeforeReset;


    double targetHeading;
    private double prevLltx = 0.0;  // previous lltx for derivative-based limelight latency prediction

    // PID Coefficients
    // Note: Since we are using Radians, the error is small (e.g., 0.5 rads).
    // You might need a higher P than 0.35 if it's sluggish.
    // Try P = 0.8 or higher if it doesn't move fast enough.
    public static double P = 0.5, secondP = 1, I = 0, D = 0.05, F = 1.16;
    public static double ll_P = 0, ll_I = 0, ll_D = 0, ll_F = 1.5;
    public static double PREDICTION_LOOKAHEAD_S = 0.030;  // 30ms control hub latency compensation
    public static double SOF_TURRET_TOLERANCE_DEG = 5.0;  // "close enough" threshold for isNearSetPoint
    public static double LL_SOF_THRESHOLD_DEG = 25.0;    // only blend when |lltx| is under this (degrees)
    public static double LIMELIGHT_LATENCY_S  = 0.050;  // Limelight 3A hardware latency to predict forward
    public static double LOOP_TIME_S          = 0.020;  // assumed loop period for lltx derivative (seconds)

    // --- MECHANICAL ROTATION LIMITS (cable anti-tangle) ---
    // Measured physically: turret can safely rotate from -270° to +190° (0 = front of robot,
    // positive = counter-clockwise). Total travel ~460°, i.e. more than one full turn, thanks
    // to cable slack — but going past either end will strain/tangle the cables.
    public static double TURRET_MIN_DEG = -270.0;
    public static double TURRET_MAX_DEG = 190.0;
    // Hardware Constants
    double gearRatio = 5.30;
    double TicksPerRev = 145.1; // Motor internal PPR

    public enum TurretState {
        IDLE,
        FULL_LIMELIGHT,
        FULL_PINPOINT,
        MIXED,
        SHOOT_ON_THE_FLY,
    }
    private static TurretState currentTurretState = TurretState.IDLE;

    public Turret(HardwareMap hwMap, Follower flwr) {
        motorTureta = hwMap.get(DcMotorEx.class, "motorTureta");

        // CHECK THIS: Ensure Positive Power = Counter-Clockwise rotation
        motorTureta.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTureta.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorTureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretController = new PIDFController(P, I, D, F);

        // Pedro uses its own Pose class, be careful not to mix up Point/Pose classes
        goalPose = (alliance == Alliance.RED) ? redGoalPose : blueGoalPose;
        follower = flwr;

        targetGoalPose = new Pose2d(this.goalPose.getX(), this.goalPose.getY(), 0);

        turretController.setTolerance(0);
    }

    public void setTurretState(TurretState state) {
        currentTurretState = state;
    }

    public TurretState getCurrentTurretState(){ return currentTurretState; }
    public double getPower(){ return motorTureta.getPower(); }
    public double getSetPoint(){ return turretController.getSetPoint(); }
    public double getCurrentPower(){
        return power;}

    void update() {


        switch (currentTurretState){
            case IDLE:
                turretController.setPIDF(2, I, D, F);
                lltx = 0;
                llty = 0;
                llta = 0;
                power = turretController.calculate(getTurretHeading(), 0);
                motorTureta.setPower(power);
                break;

            case FULL_LIMELIGHT:
                if(lltx < 5){
                    turretController.setPIDF(0,0,0,1.5);
                }
                else{
                    turretController.setPIDF(ll_P, ll_I, ll_D, ll_F);
                }

                turretController.setSetPoint(Math.toRadians(-lltx));

                power = turretController.calculate(0);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                }
                else{
                    motorTureta.setPower(power);
                }
                break;

            case FULL_PINPOINT:
                turretController.setPIDF(P, I, D, F);
                // 1. Get Robot Position & Heading from Pedro
                Pose robotPose = follower.getPose();

                // 2. Calculate the "Field Heading" (Angle from Robot to Goal globally)
                double targetFieldHeading = Math.atan2(targetGoalPose.getY() - robotPose.getY(),
                        targetGoalPose.getX() - robotPose.getX());

                // 3. Calculate "Target Local Heading"
                // This is where the turret needs to be relative to the robot chassis.
                // Formula: Field_Angle - Robot_Body_Angle
                targetHeading = targetFieldHeading - robotPose.getHeading();

                // 4. Get "Current Local Heading" from Encoder
                double currentLocalHeading = getTurretHeading(); // returns Radians

                // 5. Find the reachable absolute heading (within mechanical limits) closest to
                // the turret's current position, among all headings equivalent to targetHeading
                // (targetHeading + k*360°). This replaces the naive angleWrap "shortest path"
                // approach, which could command a path that goes past the cable limit.
                double constrainedTarget = constrainToMechanicalLimits(targetHeading, currentLocalHeading);
                double error = constrainedTarget - currentLocalHeading;



                // 6. PID Calculation
                // We calculate power to drive 'error' to 0.
                // Note: PIDFController.calculate(measured, setpoint)
                // We pass 0 as measured and error as setpoint (or vice versa depending on sign)
                // Effectively: P * error
                power = turretController.calculate(0, error);

                motorTureta.setPower(power);
                break;

            case MIXED:
                turretController.setPIDF(P, I, D, F);

                // Primary: FULL_PINPOINT odometry heading (same math as FULL_PINPOINT case)
                Pose mixedPose = follower.getPose();
                double mixedFieldHeading = Math.atan2(
                        targetGoalPose.getY() - mixedPose.getY(),
                        targetGoalPose.getX() - mixedPose.getX());
                targetHeading = mixedFieldHeading - mixedPose.getHeading();

                // Secondary: limelight fine-trim (10%) on top — no state switch, just nudge
                if (llta > 0) {
                    if(lltx < 6){
                        turretController.setPIDF(ll_P, ll_I, ll_D, ll_F);
                    }
                    if (Math.abs(lltx) < LL_SOF_THRESHOLD_DEG) {
                        targetHeading += Math.toRadians(-lltx);
                    }
                }

                // pid controller anti-tangle: pick the closest reachable heading within
                // the turret's mechanical rotation limits (same logic as FULL_PINPOINT)
                double currentMixedHeading = getTurretHeading();
                double mixedConstrainedTarget = constrainToMechanicalLimits(targetHeading, currentMixedHeading);
                double mixedError = mixedConstrainedTarget - currentMixedHeading;

                power = turretController.calculate(0, mixedError);
                motorTureta.setPower(power);
                break;

            case SHOOT_ON_THE_FLY:
                turretController.setPIDF(P, I, D, F);

                Pose sofPose = follower.getPose();
                double vRxS = follower.getVelocity().getXComponent();
                double vRyS = follower.getVelocity().getYComponent();

                // Latency-compensated robot position (accounts for ~30ms control hub loop)
                double predX = sofPose.getX() + vRxS * PREDICTION_LOOKAHEAD_S;
                double predY = sofPose.getY() + vRyS * PREDICTION_LOOKAHEAD_S;

                // Direction from predicted position to goal
                double sofDX = targetGoalPose.getX() - predX;
                double sofDY = targetGoalPose.getY() - predY;
                double sofDist = Math.sqrt(sofDX * sofDX + sofDY * sofDY);

                // Ball horizontal exit speed (in/s): K × flywheelTicks/s × cos(hoodAngle)
                double cosHood = Math.cos(Math.toRadians(Launcher.currentHoodAngleDeg));
                double vBallH = Launcher.K_LAUNCHER * Launcher.baseTargetVelocity * cosHood;//this is the 2d plane horizontal ball exit velocity

                // Compensated shot vector: ball must exit at this field-centric velocity
                // so that (V_shot_robot + V_robot) = V_ideal_to_goal
                double sofShotX = (sofDX / sofDist) * vBallH - vRxS;
                double sofShotY = (sofDY / sofDist) * vBallH - vRyS;

                // New turret heading: field-centric angle → local (robot-relative), radians
                double sofFieldHeading = Math.atan2(sofShotY, sofShotX);
                targetHeading = sofFieldHeading - sofPose.getHeading();
                // --- Limelight blend (10%) with latency prediction ---
                // SOF handles 90% (motion compensation). Limelight trims the remaining 10%
                // once it locks onto the target, accounting for Limelight 3A hardware latency.
                if (llta > 0) {
                    // The physical goal direction in field frame
                    double physicalFieldHeading = Math.atan2(sofDY, sofDX);

                    // Lead angle: how far the digital goal is offset from the physical goal
                    // This is the angular offset Limelight *should* see when perfectly aimed
                    double leadAngle = sofFieldHeading - physicalFieldHeading;  // radians

                    // Convert lead angle to degrees — this is the ideal lltx, not zero
                    double idealLltx = Math.toDegrees(leadAngle);

                    double dlltxPerSec = (lltx - prevLltx) / LOOP_TIME_S;
                    dlltxPerSec = Math.max(-500, Math.min(500, dlltxPerSec));
                    double lltxPredicted = lltx + dlltxPerSec * LIMELIGHT_LATENCY_S;

                    // Error = how far Limelight is from where it *should* be pointing
                    double llError = lltxPredicted - idealLltx;

                    if (Math.abs(llError) < LL_SOF_THRESHOLD_DEG) {
                        // positive llError = turret is right of digital goal → rotate left
                        targetHeading += Math.toRadians(-llError);
                    }
                }
                prevLltx = lltx;  // always update for next loop

                //angle wrapping custom for mechanical limitation
                if(targetHeading>Math.toRadians(90)){targetHeading -= 2*Math.PI;}
                if(targetHeading<Math.toRadians(-270)){targetHeading += 2*Math.PI;}

                // pid controller handles antistrangulation byitself (no extra logic needed)
                double sofError = targetHeading - getTurretHeading();

                power = turretController.calculate(0, sofError);
                motorTureta.setPower(power);

                // Flywheel speed compensation: back-convert |V_shot| to ticks/s for launcher PIDF
                double newBallH = Math.sqrt(sofShotX * sofShotX + sofShotY * sofShotY);
                if (Launcher.K_LAUNCHER > 0.001 && cosHood > 0.01) {
                    requiredSpeed = newBallH / (Launcher.K_LAUNCHER * cosHood);
                }
                break;
        }
    }

    /**
     * Converts motor ticks to Local Turret Radians (0 is front of robot)
     */
    public double getTurretHeading() {
        double ticks = motorTureta.getCurrentPosition();
        // Formula: (Ticks / Total_Ticks_Per_Rev) * 2PI
        return (ticks / (TicksPerRev * gearRatio)) * 2 * Math.PI;
    }

    /**
     * Normalizes an angle to be between -PI and +PI
     * This ensures the turret takes the shortest path.
     */
    private double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Anti-tangle: given a desired (unbounded) heading and the turret's current absolute
     * position (from the encoder), finds the closest reachable absolute heading — among all
     * angles equivalent to targetHeading (targetHeading + k*360°) — that stays within the
     * turret's mechanical rotation limits [TURRET_MIN_DEG, TURRET_MAX_DEG].
     *
     * This replaces plain angleWrap()'s "mathematically shortest path" with a "shortest path
     * that won't strain the cables" — if the shortest path would exceed the limit, this picks
     * the next valid wrap-around instead, or clamps to the nearest limit if truly none exist.
     */
    private double constrainToMechanicalLimits(double targetHeading, double currentLocalHeading) {
        double minRad = Math.toRadians(TURRET_MIN_DEG);
        double maxRad = Math.toRadians(TURRET_MAX_DEG);

        double normalizedTarget = angleWrap(targetHeading); // bring into [-PI, PI] first

        double best = Double.NaN;
        double bestDist = Double.MAX_VALUE;

        // Check a handful of full-turn offsets — enough to cover a >360° travel range.
        for (int k = -2; k <= 2; k++) {
            double candidate = normalizedTarget + k * 2 * Math.PI;
            if (candidate >= minRad && candidate <= maxRad) {
                double dist = Math.abs(candidate - currentLocalHeading);
                if (dist < bestDist) {
                    bestDist = dist;
                    best = candidate;
                }
            }
        }

        if (Double.isNaN(best)) {
            // No equivalent angle falls inside the allowed range (shouldn't normally happen
            // given a >360° range) — fall back to clamping the raw target to the nearest limit.
            best = Math.max(minRad, Math.min(maxRad, normalizedTarget));
        }

        return best;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public boolean isNearSetPoint() {
        return Math.abs(angleWrap(targetHeading - getTurretHeading()))
                < Math.toRadians(SOF_TURRET_TOLERANCE_DEG);
    }

    @Override
    public void periodic() {
        robotAngle = follower.getPose().getHeading();

        update();
    }
}