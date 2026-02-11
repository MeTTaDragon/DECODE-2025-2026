package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;

public class LauncherStateCommand extends InstantCommand {
    private Launcher.LauncherState state;
    private Launcher launcher;

    public LauncherStateCommand(Launcher launcherSubsystem, Launcher.LauncherState state) {
        this.launcher = launcherSubsystem;
        this.state = state;
        addRequirements(launcherSubsystem);
    }

    //set the parameter as a runnable(a lambda) for custom calls
    public LauncherStateCommand(Runnable runnable){
        runnable.run();
    }

    @Override
    public void initialize() {
        launcher.setCurrentLauncherState(state);
    }
}
