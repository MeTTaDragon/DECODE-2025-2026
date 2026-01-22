package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range; // Import Range for clipping power
// A nu se da merge pt ca nu este facut ca fisier decat de test
@TeleOp(name = "Test Motor Dpad", group = "Testing")
public class testmotor extends LinearOpMode {

    private DcMotor myMotor;

    // Variable to hold the current power level
    double currentPower = 0.0;

    // Variables to track button states from the previous loop
    boolean lastUp = false;
    boolean lastDown = false;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        myMotor = hardwareMap.get(DcMotor.class, "motor_1");
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "D-pad Up (+0.1), D-pad Down (-0.1)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. Read current button states
            boolean isUpPressed = gamepad1.dpad_up;
            boolean isDownPressed = gamepad1.dpad_down;

            // 2. Logic: Only change power if button is pressed NOW but wasn't BEFORE

            // Check D-pad Up
            if (isUpPressed && !lastUp) {
                currentPower += 0.1;
            }

            // Check D-pad Down
            if (isDownPressed && !lastDown) {
                currentPower -= 0.1;
            }

            // 3. Update the 'last' variables for the next loop
            lastUp = isUpPressed;
            lastDown = isDownPressed;

            // 4. Safety: Ensure power stays between -1.0 and 1.0
            // Sometimes math creates numbers like 1.0000001, so we clip it.
            currentPower = Range.clip(currentPower, -1.0, 1.0);

            // 5. Send power to motor
            myMotor.setPower(currentPower);

            // 6. Telemetry
            // (%.1f formats the number to 1 decimal place, e.g., "0.5")
            telemetry.addData("Motor Power", "%.1f", currentPower);
            telemetry.addData("Motor Position", myMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}