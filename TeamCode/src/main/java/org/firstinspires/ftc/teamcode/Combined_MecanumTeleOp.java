// Please note that this code uses the Logitech F310 as its controller
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp // Without this, this file will not show in the TeleOp section of the REV Driver Hub. Note that REV Driver Hub and REV Driver Station are synonymous.
public class Combined_MecanumTeleOp extends LinearOpMode {
    double accelerationFactor = 0.15; // Sets the default speed to 15% (0.15).

    // Defines 4 Mecanum Wheel Motors, and then the Viper Slide Motor.
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorViperSlide;

    // Creates IMU that is set to imu.
    IMU imu;

    boolean isFieldCentric = true; // Sets the default to field-centric mode when CombinedMecanumTeleOp is initialized on the REV Driver Hub.

    @Override
    public void runOpMode() throws InterruptedException {
        // Declares our motors using ID's that match the configuration on the REV Control Hub
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // Front Left Motor
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); // Back Left Motor
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // Front Right Motor
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); // Back Right Motor
        motorViperSlide = hardwareMap.dcMotor.get("motorViperSlide"); // Viper Slide Motor

        // Sets the viper slide motor to use encoders
        motorViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resets encoder value of viper slide motor to 0
        motorViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the zero power behavior to BRAKE for all motors
        motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorViperSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors since we are using mecanum wheels
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieves the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjusts the orientation parameters to match the robot (note that IMU is set to imu)
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Variable that stores the speed of the viper slide motor, modify based on what works best for your robot
        double motorViperSlideSpeed = 0.8;

        // Adds telemetry to the Driver Station
        telemetry.addData("Status", "Initialized"); // Adds Initialized Status
        telemetry.addData("Mode", "Field-Centric"); // Since the default mode is Field-Centric, sets Field-Centric to be the mode that is added to REV Driver Hub
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Resets imu at the start of code
        imu.resetYaw();

        // Run until the end of the match (driver presses STOP)
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Toggle control mode on left joystick button press
            if (gamepad1.left_stick_button) {
                isFieldCentric = !isFieldCentric; // Toggle the mode
                sleep(200); // Small delay to avoid multiple toggles

                if (isFieldCentric) { // Activates when the mode is Field Centric
                    telemetry.addData("Mode", "Field-Centric"); // Report the mode change to Field-Centric on Driver Hub

                } else { // Activates when the mode is Field Centric
                    telemetry.addData("Mode", "Robot-Centric"); // Report the mode change to Robot-Centric on Driver Hub
                }
                telemetry.update(); // Adds the mode telemetry to REV Driver Hub
            }

            // int position = motorViperSlide.getCurrentPosition(); // Unused variable that stores the current encoder position for the viper slide motor.
            // Note that when it is a non-zero number, it will be negative since moving the viper slide motor counterclockwise (which results in a negative position value) moves the viper slide up.

            // Get raw values from the gamepad
            double y = -gamepad1.left_stick_y; // Negative because the gamepad's y-axis is inverted
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            int small_pole = -1510; // The encoder position for the small pole
            int medium_pole = -2519; // The encoder position for the medium pole
            int large_pole = -3780; // The encoder position for the large pole
            // Note that all encoder positions are negative because counterclockwise moves the slide up

            // Controls for vipers slide using presets
            // When the "a" button is pressed, the viper slide motor will move to the bottom (0) using encoders
            if (gamepad1.a) {
                motorViperSlide.setTargetPosition(0); // Sets target position to 0
                motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This causes the motor to start moving to encoder position 0 while the "a" button is pressed and held down.
                motorViperSlide.setPower(motorViperSlideSpeed); // This sets the speed at which motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); will run at.
            }

            // When the "x" button is pressed, the viper slide motor will move to the small pole position using encoders
            else if (gamepad1.x) {
                motorViperSlide.setTargetPosition(small_pole); // Sets target position to small_pole
                motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This causes the motor to start moving to encoder position small_pole while the "x" button is pressed and held down.
                motorViperSlide.setPower(motorViperSlideSpeed); // This sets the speed at which motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); will run at.
            }

            // When the "y" button is pressed, the viper slide motor will move to the medium pole position using encoders
            else if (gamepad1.y) {
                motorViperSlide.setTargetPosition(medium_pole); // Sets target position to medium_pole
                motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This causes the motor to start moving to encoder position medium_pole while the "y" button is pressed and held down.
                motorViperSlide.setPower(motorViperSlideSpeed); // This sets the speed at which motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); will run at.
            }

            // When the "b" button is pressed, the viper slide motor will move to the large pole position using encoders
            else if (gamepad1.b) {
                motorViperSlide.setTargetPosition(large_pole); // Sets target position to large_pole
                motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This causes the motor to start moving to encoder position large_pole while the "b" button is pressed and held down.
                motorViperSlide.setPower(motorViperSlideSpeed); // This sets the speed at which motorViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); will run at.
            }

            // Control motorViperSlide using dpad (no presets)
            // When the down dpad is pressed, the viper slide motor will move down using encoders
            else if (gamepad1.dpad_down) {
                if (motorViperSlide.getCurrentPosition() < -100) { // Checks if the motor is at the bottom to make sure it cannot run past it
                    motorViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // While the down dpad button is pressed, the viper slide will move down using the speed of motorViperSlideSpeed. Holding it down will cause it to move down for a larger distance more smoothly and quickly.
                    motorViperSlide.setPower(motorViperSlideSpeed); // Runs clockwise at 80% speed, since counterclockwise moves the viper slide up
                }
            }

            // When the up dpad is pressed, the viper slide motor will move up using encoders
            else if (gamepad1.dpad_up ) {
                if (motorViperSlide.getCurrentPosition() > -4200) { // Checks if the motor is nearly at the top to make sure it cannot run past it
                    motorViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // While the up dpad button is pressed, the viper slide will move down using the speed of motorViperSlideSpeed. Holding it down will cause it to move up for a larger distance more smoothly and quickly.
                    motorViperSlide.setPower(-motorViperSlideSpeed); // Runs counterclockwise at 80% speed, since counterclockwise moves the viper slide up
                }
            }

            // When the viper slide motor is not moving, it will brake, causing it to hold its position
            else {
                motorViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Runs to make sure motor is still not running
                motorViperSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); // Brakes when not moving
                motorViperSlide.setPower(0); // Sets power to 0, thus preventing it from running
            }

            // Creates three variables that are used for the Mecanum wheel calculations for Robot-Centric Mode
            double forward, sideways, rotation;

            // Convert the raw x and y values to robot-centric forward and sideways velocities for easier understanding
            forward = y;
            sideways = x;
            rotation = rx;

            // Use the LT value as an acceleration factor for all mecanum wheel movement.
            // LT value is between 0.15 (not pressed) and 1 (fully pressed).
            double lt = gamepad1.left_trigger;
            double ltSpeed = accelerationFactor + (1 - accelerationFactor) * lt;

            // Reset the yaw angle to 0 degrees when the "Back" button is pressed. Is used for Field-Centric mode, but can be activated during Robot-Centric Mode for Field-Centric mode/
            if (gamepad1.back) {
                imu.resetYaw();
            }

            // Resets encoder value of viper slide motor to 0 when the right joystick button is pressed. Can be used in both Field-Centric and Robot-Centric mode.
            if (gamepad1.right_stick_button) {
                motorViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // If the robot is in Field-Centric Mode, the robot will NOT have a head (meaning that the robot's controls will change based off the direction it is facing). What direction is forward can be done be resetting the yaw angle to 0 degrees (through pressing gamepad.back).
            if (isFieldCentric) {

                // Calculate motor powers using mecanum drive kinematics
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the robot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
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

                // Display motorViperSlide encoder position
                telemetry.addData("Mode:", "Field-Centric"); // Displays current mode (Field-Centric)
                telemetry.addData("Front Left Power", frontLeftPower); // Displays power of the front left mecanum wheel
                telemetry.addData("Back Left Power", backLeftPower); // Displays power of the back left mecanum wheel
                telemetry.addData("Front Right Power", frontRightPower); // Displays power of the front right  mecanum wheel
                telemetry.addData("Back Right Power", backRightPower); // Displays power of the back right mecanum wheel
            }

            // If the robot is in Robot-Centric Mode, the robot will WILL have a head (meaning that the robot's controls WILL change based off the direction it is facing). You can still reset the yaw angle to 0 degrees (through pressing the back button) in Robot-Centric, then switch to Field-Centric mode (via the left joystick button) and be able to use that yaw angle.
            else {
                // Calculate motor powers using mecanum drive kinematics
                double frontLeftPower = (forward + sideways + rotation) * ltSpeed;
                double frontRightPower = (forward - sideways - rotation) * ltSpeed;
                double backLeftPower = (forward - sideways + rotation) * ltSpeed;
                double backRightPower = (forward + sideways - rotation) * ltSpeed;

                // Set motor powers
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);

                telemetry.addData("Mode:", "Robot-Centric"); // Displays current mode (Robot-Centric)
                telemetry.addData("Front Left Power", frontLeftPower); // Displays power of the front left mecanum wheel
                telemetry.addData("Back Left Power", backLeftPower); // Displays power of the back left mecanum wheel
                telemetry.addData("Front Right Power", frontRightPower); // Displays power of the front right  mecanum wheel
                telemetry.addData("Back Right Power", backRightPower); // Displays power of the back right mecanum wheel
            }
            telemetry.addData("Speed", ltSpeed); // Displays speed of robot mecanum wheel movement using left trigger (between 0.15 and 1)
            telemetry.addData("ViperSlide Position", motorViperSlide.getCurrentPosition());  // Displays motorViperSlide encoder position
            telemetry.addData("ViperSlide Mode", motorViperSlide.getMode()); // Displays motorViperSlide mode
            telemetry.update(); // Adds telemetry to REV Driver Hub
        }
    }
}