package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;

public class IntakeStateCommand extends InstantCommand {
    Intake intake;
    Intake.IntakeState state;

    public IntakeStateCommand(Intake intake, Intake.IntakeState state) {
        this.intake = intake;
        this.state = state;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeState(state);
        if(state == Intake.IntakeState.INTAKE){
            intake.setintakePos(Intake.servoPosDown);
        }
        else if(state == Intake.IntakeState.IDLE){
            intake.setintakePos(Intake.servoPosUp);
        }
    }
}
