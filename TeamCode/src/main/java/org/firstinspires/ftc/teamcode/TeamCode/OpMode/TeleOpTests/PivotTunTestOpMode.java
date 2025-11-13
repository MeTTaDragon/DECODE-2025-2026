package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOpTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun; // Importam subsistemul PivotTun

/**
 * Acest OpMode este folosit pentru a testa subsistemul 'PivotTun' folosind FTC Dashboard.
 *
 * DASHBOARD:
 * Te poți conecta la FTC Dashboard pentru a modifica variabilele din clasa 'PivotTun':
 * 1. 'TARGET_POSITION': Setează poziția țintă (în ticks) pentru motor.
 * 2. 'PIVOT_POWER': Setează puterea maximă folosită pentru a ajunge la țintă.
 * 3. 'tolerance': Setează toleranța de poziție a motorului.
 */
@Config // Asigură-te că adnotarea @Config este prezentă
@TeleOp(name = "Testare PivotTun (Dashboard Control)", group = "Test")
public class PivotTunTestOpMode extends LinearOpMode {

    private PivotTun pivotTun;



    @Override
    public void runOpMode() throws InterruptedException {
        // Inițializează telemetria pentru a funcționa atât pe telefon, cât și pe Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Inițializează subsistemul PivotTun
        pivotTun = new PivotTun(hardwareMap);
        pivotTun.init(); // Setează motorul în RUN_TO_POSITION și ținta la 0

        telemetry.addLine("Robotul este gata de start.");
        telemetry.addLine("--- CONTROALE ---");
        telemetry.addLine("Controlul se face EXCLUSIV din FTC Dashboard.");
        telemetry.addLine("Deschide clasa 'PivotTun' în Dashboard.");
        telemetry.addLine("Modifică 'TARGET_POSITION', 'PIVOT_POWER', și 'tolerance'.");
        telemetry.addLine("-----------------");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Apelează periodic() în buclă.
            // Această metodă va citi valorile statice (TARGET_POSITION, PIVOT_POWER)
            // actualizate de Dashboard și le va aplica motorului.
            pivotTun.periodic();

            // Afișează telemetrie cu starea curentă și puterile
            telemetry.addData("Target Position (din PivotTun)", PivotTun.getTARGET_POSITION());
            telemetry.addData("Current Position", PivotTun.getCurrentPosition());
            telemetry.addData("Pivot Power (din PivotTun)", PivotTun.getPIVOT_POWER());
            telemetry.addData("Tolerance (din PivotTun)", PivotTun.getTolerance());
            telemetry.update();
        }

        // Când OpMode-ul se oprește, bucla se termină și periodic() nu mai este apelat.
        // Motorul ar trebui să se oprească sau să mențină poziția datorită
        // setării ZeroPowerBehavior.BRAKE.
    }
}