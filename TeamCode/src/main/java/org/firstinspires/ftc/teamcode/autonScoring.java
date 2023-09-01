// Please note, no RoadRunner has been implemented into this Auton so far.
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.ServoCommands;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Autonomous(name = "Picking up cones and scoring auton")

public class autonScoring extends LinearOpMode{

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorRightViperSlide;
    DcMotor motorLeftViperSlide;
    Servo servoLeftClaw;
    Servo servoRightClaw;
    IMU imu;
    int small_pole = -1710; // The encoder position for the small pole
    int medium_pole = -2719; // The encoder position for the medium pole
    int large_pole = -4087; // The encoder position for the large pole
    @Override
    public void runOpMode() throws InterruptedException{
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorRightViperSlide = hardwareMap.dcMotor.get("motorRightViperSlide");
        motorLeftViperSlide = hardwareMap.dcMotor.get("motorLeftViperSlide");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        motorRightViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRightViperSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorLeftViperSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        waitForStart();
        imu.resetYaw();
        if (isStopRequested()) return;
        double motorViperSlideSpeed = 0.8;
        servoLeftClaw.setPosition(0);
        servoRightClaw.setPosition(0);
        int b = 1;
        if (b == 1){
            smallJunction();
        } else if (b == 2) {
            mediumJunction();
        } else if (b==3){
            largeJunction();
        }
    }

    private void largeJunction() {
        int a = 1;
        while (a<5){
            pickUpCone(a);
            sleep(10000);
            motorLeftViperSlide.setPower(0.8);
            motorLeftViperSlide.setTargetPosition(medium_pole);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            a +=1;
        }
    }

    private void mediumJunction() {
        int a = 1;
        while (a<5){
            pickUpCone(a);
            sleep(10000);
            motorLeftViperSlide.setPower(0.8);
            motorLeftViperSlide.setTargetPosition(medium_pole);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            a +=1;
        }
    }

    public void smallJunction() {
        int a = 1;
        while (a<6) {
            pickUpCone(a);
            sleep(10000);
            motorLeftViperSlide.setPower(0.8);
            motorLeftViperSlide.setTargetPosition(medium_pole);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            a += 1;
        }
    }

    public void pickUpCone(int coneNumber) {
        int conePosition = 1;
        if (coneNumber == 1){
            conePosition = -100;
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoLeftClaw.setPosition(1);
            servoRightClaw.setPosition(1);
            conePosition -= 1000;
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        else if (coneNumber == 2){
            conePosition = -200;
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoLeftClaw.setPosition(1);
            servoRightClaw.setPosition(1);
            conePosition -= 1000;
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (coneNumber == 3){
            conePosition = -300;
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoLeftClaw.setPosition(1);
            servoRightClaw.setPosition(1);
            conePosition -= 1000;
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (coneNumber == 4){
            conePosition = -400;
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoLeftClaw.setPosition(1);
            servoRightClaw.setPosition(1);
            conePosition -= 1000;
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (coneNumber == 5){
            conePosition = -500;
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoLeftClaw.setPosition(1);
            servoRightClaw.setPosition(1);
            conePosition -= 1000;
            motorRightViperSlide.setPower(0.8);
            motorLeftViperSlide.setPower(0.8);
            motorRightViperSlide.setTargetPosition(conePosition);
            motorLeftViperSlide.setTargetPosition(conePosition);
            motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //write this once u figure out the encoders for the cones
    }
}
