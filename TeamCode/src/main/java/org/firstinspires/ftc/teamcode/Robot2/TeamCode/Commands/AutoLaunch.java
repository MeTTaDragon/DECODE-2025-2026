package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperClose;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperOpen;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

public class AutoLaunch extends CommandBase {
    private final Launcher launcher;
    private final Intake intake;
    private final Turret turret;
    private LimelightSubsystem limelight;
    public static boolean trackingEnabled = false;
    public AutoLaunch(Launcher launcher, Intake intake, Turret turret, LimelightSubsystem limelight, boolean trackingEnabled) {
        this.launcher = launcher;
        this.intake = intake;
        this.turret = turret;
        this.limelight = limelight;
        this.trackingEnabled = trackingEnabled;
        addRequirements(launcher, turret, intake, limelight);
    }
    public AutoLaunch(Launcher launcher, Intake intake, Turret turret) {
        this.launcher = launcher;
        this.intake = intake;
        this.turret = turret;
        addRequirements(launcher, turret, intake);
    }

    @Override
    public void initialize() {
        launcher.setCurrentLauncherState(Launcher.LauncherState.SHOOTING);
        turret.setTurretState(Turret.TurretState.FULL_PINPOINT);
    }

    @Override
    public void execute(){
        if (launcher.isVelocityReached()){
            if(trackingEnabled)
            {
                limelight.setMode(LimelightSubsystem.LimelightMode.BASKET);
                turret.setTurretState(Turret.TurretState.FULL_LIMELIGHT);
                trackingEnabled = false;
            }
            launcher.setStopperPose(stopperOpen);

            if(launcher.isStopperOpen()){
                intake.setIntakeState(Intake.IntakeState.REVERSE);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE);
        turret.setTurretState(Turret.TurretState.IDLE);
        launcher.setStopperPose(stopperClose);
        intake.setIntakeState(Intake.IntakeState.IDLE);
        limelight.setMode(LimelightSubsystem.LimelightMode.PAUSE);
    }

}
