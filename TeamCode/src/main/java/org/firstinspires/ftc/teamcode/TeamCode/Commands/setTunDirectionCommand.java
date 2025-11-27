package org.firstinspires.ftc.teamcode.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;

public class setTunDirectionCommand extends CommandBase {
    Tun tun;
    PivotTun pivotTun;
    LimelightSubsystem ll;
    boolean llConnected = false;
    int pivotTargetPosition;
    Tun.tunState tunState;

    /**
     * Constructs a new setTunDirectionCommand.
     * This command is responsible for setting the state of the Tun and the position of the pivot (PivotTun) simultaneously.
     *
     * @param tun                 The Tun subsystem that this command will control.
     * @param pivotTun            The PivotTun subsystem that this command will control.
     * @param pivotTarget         The target encoder position for the pivot motor.
     */
    public setTunDirectionCommand(Tun tun, PivotTun pivotTun, int pivotTarget) {
        this.tun = tun;
        this.pivotTun = pivotTun;
        this.pivotTargetPosition = pivotTarget;
        addRequirements(tun, pivotTun);
    }

    /**
     * Constructs a new setTunDirectionCommand.
     * This command is responsible for setting the state of the Tun and the position of the pivot (PivotTun) simultaneously.
     *
     * @param tun                 The Tun subsystem that this command will control.
     * @param pivotTun            The PivotTun subsystem that this command will control.
     * @param ll                  The LimelightSubsystem to adjust the servo position based on pivot direction.
     * @param pivotTargetPosition The target encoder position for the pivot motor.
     */
    public setTunDirectionCommand(Tun tun, PivotTun pivotTun, LimelightSubsystem ll, int pivotTargetPosition) {
        this.tun = tun;
        this.pivotTun = pivotTun;
        this.pivotTargetPosition = pivotTargetPosition;
        this.ll = ll;
        addRequirements(tun, pivotTun, ll);

        llConnected = true;
    }


    @Override
    public void initialize() {
        pivotTun.setPivotPosition(pivotTargetPosition);

        if(pivotTargetPosition > 0) {
            tun.setTunState(Tun.tunState.FORWARD);
        } else if (pivotTargetPosition < 0) {
            tun.setTunState(Tun.tunState.REVERSE);
        } else {
            tun.setTunState(Tun.tunState.IDLE);
        }


        //TODO: verifica directia motorului sa fie pe directii opuse pivotu cu camera
        if(llConnected) {
            if (pivotTargetPosition > 0) ll.setLLServoState(LimelightSubsystem.LLServoState.BACK);
            else ll.setLLServoState(LimelightSubsystem.LLServoState.FRONT);
        }
    }
}
