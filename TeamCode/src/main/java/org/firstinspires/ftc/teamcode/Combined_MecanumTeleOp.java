package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Combined_MecanumTeleOp extends LinearOpMode {
    Field_Centric_MecanumTeleOp fieldCentricModeOpMode;
    Robot_Centric_MecanumTeleOp robotCentricModeOpMode;

    @Override
    public void runOpMode() throws InterruptedException {
        fieldCentricModeOpMode = new Field_Centric_MecanumTeleOp();
        robotCentricModeOpMode = new Robot_Centric_MecanumTeleOp();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_button) {
                if (fieldCentricModeOpMode.opModeIsActive()) {
                    fieldCentricModeOpMode.stop();  // Stop the current mode
                    robotCentricModeOpMode.runOpMode();  // Start the other mode
                } else {
                    robotCentricModeOpMode.stop();  // Stop the current mode
                    fieldCentricModeOpMode.runOpMode();  // Start the other mode
                }

                sleep(200); // Small delay to avoid registering multiple presses
            }
        }
    }
}
