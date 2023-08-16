package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Combined_MecanumTeleOp extends LinearOpMode {
    double accelerationFactor = 0.15; // Sets the default speed to 15% (0.15).

    //Defines 4 Mecanum Wheel Motors, and then the Viper Slide Motor
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorViperSlide;
    IMU imu;

    boolean isFieldCentric = true; // Sets the default to field-centric mode when CombinedMechanumTeleOp is initialized on the REV Driver Hub.

    @Override
    public void runOpMode() throws InterruptedException {
        // Declares our motors using ID's that match the configuration on the REV Control Hub
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorViperSlide = hardwareMap.dcMotor.get("motorViperSlide");

        //Sets the viper slide motor to use encoders
        motorViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the zero power behavior to BRAKE for all motors
        motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorViperSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieves the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjusts the orientation parameters to match the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Variables that stores the speed of the viper slide motor
        double motorViperSlideSpeed = 0.8;

        //Adds telemetry to the Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Field-Centric");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Toggle control mode on left joystick button press
            if (gamepad1.left_stick_button) {
                isFieldCentric = !isFieldCentric; // Toggle the mode
                sleep(200); // Small delay to avoid multiple toggles

                if (isFieldCentric) {
                    telemetry.addData("Mode", "Field-Centric"); // Report the mode change to Field-Centric on Driver Hub
                } else {
                    telemetry.addData("Mode", "Robot-Centric"); // Report the mode change to Robot-Centric on Driver Hub
                }
                telemetry.update();
            }
            int position = motorViperSlide.getCurrentPosition();
            // Get raw values from the gamepad
            double y = -gamepad1.left_stick_y; // Negative because the gamepad's y-axis is inverted
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            int small_pole = -1510; // The encoder position for the small pole
            int medium_pole = -2519; // The encoder position for the medium pole
            int large_pole = -3450; // The encoder position for the large pole
            // Note that all encoder positions are negative because counterclockwise moves the slide up

            // When the "a" button is pressed, the viper slide motor will move to the bottom (0) using encoders
            if (gamepad1.a){
                motorViperSlide.setTargetPosition(0);
                motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorViperSlide.setPower(motorViperSlideSpeed);
            }

            // When the "x" button is pressed, the viper slide motor will move to the small pole position using encoders
            else if (gamepad1.x){
                motorViperSlide.setTargetPosition(small_pole);
                motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorViperSlide.setPower(motorViperSlideSpeed);
            }

            // When the "y" button is pressed, the viper slide motor will move to the medium pole position using encoders
            else if (gamepad1.y){
                motorViperSlide.setTargetPosition(medium_pole);
                motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorViperSlide.setPower(motorViperSlideSpeed);
            }

            // When the "b" button is pressed, the viper slide motor will move to the large pole position using encoders
            else if (gamepad1.b){
                motorViperSlide.setTargetPosition(large_pole);
                motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorViperSlide.setPower(motorViperSlideSpeed);
            }

            // When the down dpad is pressed, the viper slide motor will move down using encoders
            else if (gamepad1.dpad_down) {
                motorViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorViperSlide.setPower(motorViperSlideSpeed); // Runs clockwise at 20% speed

            // When the up dpad is pressed, the viper slide motor will move up using encoders
            } else if (gamepad1.dpad_up) {
                motorViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorViperSlide.setPower(-motorViperSlideSpeed); // Runs counterclockwise at 20% speed

            // When the viper slide motor is not moving, it will brake, causing it to hold its position
            } else {
                motorViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorViperSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); // Brake when not moving
                motorViperSlide.setPower(0); // Stop
            }

            // Creates three variables that are used for the Mecanum wheel calculations for both Field-Centric Mode and Robot-Centric Mode
            double forward, sideways, rotation;

            // If the robot is in Field-Centric Mode, the robot will have a head (meaning that the robot's controls will change based off the direction it is facing)
            if (isFieldCentric) {
                // Convert the raw x and y values to field-centric forward and sideways velocities
                forward = y * Math.cos(getBotHeading()) + x * Math.sin(getBotHeading());
                sideways = -y * Math.sin(getBotHeading()) + x * Math.cos(getBotHeading());
                rotation = rx;

                // Use the LT value as an acceleration factor.
                // LT value is between 0.15 (not pressed) and 1 (fully pressed).
                double lt = gamepad1.left_trigger;
                double ltSpeed = accelerationFactor + (1 - accelerationFactor) * lt;

                // Reset the yaw angle to 0 degrees when the "Back" button is pressed.
                if (gamepad1.back) {
                    imu.resetYaw();
                }

                // Calculate motor powers using mecanum drive kinematics
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator * ltSpeed;
                double backLeftPower = (rotY - rotX + rx) / denominator * ltSpeed;
                double frontRightPower = (rotY - rotX - rx) / denominator * ltSpeed;
                double backRightPower = (rotY + rotX - rx) / denominator * ltSpeed;

                // Set motor powers
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);

                // Control motorViperSlide using dpad

                // Display motorViperSlide encoder position
                telemetry.addData("Mode:", "Field-Centric");
                telemetry.addData("Speed", ltSpeed);
                telemetry.addData("Front Left Power", frontLeftPower);
                telemetry.addData("Back Left Power", backLeftPower);
                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);

            // If the robot is in Robot-Centric Mode, the robot will NOT have a head (meaning that the robot's controls will not change based off the direction it is facing). What direction is forward can be done be resetting the yaw angle to 0 degrees (through pressing gamepad.back)
            } else {
                // Convert the raw x and y values to robot-centric forward and sideways velocities
                forward = y;
                sideways = x;
                rotation = rx;

                // Use the LT value as an acceleration factor.
                // LT value is between 0.15 (not pressed) and 1 (fully pressed).
                double lt = gamepad1.left_trigger;
                double speed = accelerationFactor + (1 - accelerationFactor) * lt;

                // Reset the yaw angle to 0 degrees when the "Back" button is pressed.
                if (gamepad1.back) {
                    imu.resetYaw();
                }
                // Calculate motor powers using mecanum drive kinematics
                double frontLeftPower = (forward + sideways + rotation) * speed;
                double backLeftPower = (forward - sideways + rotation) * speed;
                double frontRightPower = (forward - sideways - rotation) * speed;
                double backRightPower = (forward + sideways - rotation) * speed;

                // Set motor powers
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);


                // Display motorViperSlide encoder position
                telemetry.addData("Mode:", "Robot-Centric");
                telemetry.addData("Speed", speed);
                telemetry.addData("Front Left Power", frontLeftPower);
                telemetry.addData("Back Left Power", backLeftPower);
                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);

            }

            // Displays motorViperSlide encoder position
            telemetry.addData("ViperSlide Position", motorViperSlide.getCurrentPosition());

            // Displays motorViperSlide mode
            telemetry.addData("ViperSlide Mode", motorViperSlide.getMode());
            telemetry.update();
        }
    }

    // Helper method to get the robot's heading (yaw) from the IMU
    private double getBotHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}