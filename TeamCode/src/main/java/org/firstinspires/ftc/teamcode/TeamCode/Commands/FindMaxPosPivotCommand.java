package org.firstinspires.ftc.teamcode.TeamCode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;

public class FindMaxPosPivotCommand extends CommandBase {
    PivotTun pivotTun;

    public FindMaxPosPivotCommand(PivotTun pivot){
        this.pivotTun = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize(){
        pivotTun.motorPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotTun.motorPivot.setPower(0.5);
    }

    @Override
    public void execute(){
        if(pivotTun.getSwitchState()){
            pivotTun.motorPivot.setPower(0);
            pivotTun.MAX_PIVOT = pivotTun.motorPivot.getCurrentPosition();

            pivotTun.motorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotTun.setPivotPosition(-2350+pivotTun.MAX_PIVOT);
            pivotTun.motorPivot.setPower(0.5);
        }

    }

    @Override
    public boolean isFinished(){
        if( pivotTun.motorPivot.getCurrentPosition() == pivotTun.motorPivot.getTargetPosition() ) {
            pivotTun.motorPivot.setPower(0);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivotTun.motorPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
