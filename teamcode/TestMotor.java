package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestMotor extends LinearOpMode {
    DcMotor Test;

    @Override
    public void runOpMode() throws InterruptedException {

        Test = hardwareMap.get(DcMotor.class, "Test");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                Test.setPower(0.5);
            }
            else if (gamepad1.b) {
                Test.setPower(-0.5);
            }
            else {
                Test.setPower(0);
            }
        }
    }
}