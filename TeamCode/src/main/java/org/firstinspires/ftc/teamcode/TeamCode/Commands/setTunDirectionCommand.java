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
     * @param tunState            The desired state for the intake (e.g., FORWARD, REVERSE, IDLE).
     */
    public setTunDirectionCommand(Tun tun, PivotTun pivotTun, Tun.tunState tunState) {
        this.tun = tun;
        this.pivotTun = pivotTun;
        this.pivotTargetPosition = pivotTargetPosition;
        this.tunState = tunState;
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
     * @param tunState            The desired state for the intake (e.g., FORWARD, REVERSE, IDLE).
     */
    public setTunDirectionCommand(Tun tun, PivotTun pivotTun, LimelightSubsystem ll, int pivotTargetPosition, Tun.tunState tunState) {
        this.tun = tun;
        this.pivotTun = pivotTun;
        this.pivotTargetPosition = pivotTargetPosition;
        this.ll = ll;
        this.tunState = tunState;
        addRequirements(tun, pivotTun, ll);

        llConnected = true;
    }


    @Override
    public void initialize() {
        tun.setTunState(tunState);
        pivotTun.setPivotPosition(pivotTargetPosition);

        //TODO: verifica directia motorului sa fie pe directii opuse pivotu cu camera
        if(llConnected) {
            if (pivotTargetPosition > 0) ll.setLLServoState(LimelightSubsystem.LLServoState.BACK);
            else ll.setLLServoState(LimelightSubsystem.LLServoState.FRONT);
        }
    }
}
