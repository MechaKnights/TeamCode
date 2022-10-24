package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Testing extends LinearOpMode {
    CRServo servo = hardwareMap.get(CRServo.class, "servo");
    CRServo servo1 = hardwareMap.get(CRServo.class, "servo1");
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0) {
                servo.setPower(0.75);
            }
            else if (gamepad1.right_trigger > 0) {
                servo.setPower(-0.75);
            }
            else {
                servo.setPower(0);
            }
            if (gamepad1.dpad_up) {
                servo1.setPower(0.7);
            }
            else if (gamepad1.dpad_down) {
                servo1.setPower(-0.7);
            }
            else if (gamepad1.dpad_left) {

            }
            else if (gamepad1.dpad_right) {

            }
            else {

            }
            if (gamepad1.a) {

            }
            else if (gamepad1.b) {

            }
            else if (gamepad1.x) {

            }
            else if (gamepad1.y) {

            }
            else {

            }
        }
    }
}
