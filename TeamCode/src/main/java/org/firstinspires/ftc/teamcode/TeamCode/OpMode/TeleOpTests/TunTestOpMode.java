package org.firstinspires.ftc.teamcode.TeamCode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun; // Importam subsistemul Tun

/**
 * Acest OpMode este folosit pentru a testa subsistemul 'Tun' folosind FTC Dashboard.
 *
 * DASHBOARD:
 * Te poți conecta la FTC Dashboard pentru a modifica:
 * 1. 'testState': Alege starea (FORWARD, REVERSE, IDLE) dintr-un meniu dropdown.
 * 2. 'TUN_POWER' (din clasa Tun): Setează puterea motoarelor.
 * 3. 'BAND_POWER' (din clasa Tun): Setează puterea benzii.
 */
@Config // Asigură-te că adnotarea @Config este prezentă
@TeleOp(name = "Testare Tun (Dashboard Control)", group = "Test")
public class TunTestOpMode extends LinearOpMode {

    private Tun tun;

    // Variabilă statică pentru a fi controlată din Dashboard
    // Aceasta va apărea ca un meniu dropdown în Dashboard
    public static Tun.tunState testState = Tun.tunState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        // Inițializează telemetria pentru a funcționa atât pe telefon, cât și pe Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Inițializează subsistemul Tun
        tun = new Tun(hardwareMap);
        tun.init(); // Setează starea inițială la IDLE

        telemetry.addLine("Robotul este gata de start.");
        telemetry.addLine("--- CONTROALE ---");
        telemetry.addLine("Controlul se face EXCLUSIV din FTC Dashboard.");
        telemetry.addLine("Modifică 'testState' (FORWARD, REVERSE, IDLE).");
        telemetry.addLine("Modifică 'TUN_POWER' și 'BAND_POWER' din clasa Tun.");
        telemetry.addLine("-----------------");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Aplică starea selectată din Dashboard în mod continuu
            // Variabila 'testState' este actualizată automat de Dashboard
            tun.setTunState(testState);

            // Afișează telemetrie cu starea curentă și puterile
            telemetry.addData("Stare Selectata (din Dashboard)", testState);
            telemetry.addData("Stare Curenta Aplicata", Tun.getCurrentTunState());
            telemetry.addData("Putere Motor (din Tun)", Tun.getTunPower());
            telemetry.addData("Putere Banda (din Tun)", Tun.getBandPower());
            telemetry.update();
        }

        // O bună practică este să oprești motoarele la final
        tun.setTunState(Tun.tunState.IDLE);
    }
}