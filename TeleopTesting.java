package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class TeleopTesting extends LinearOpMode {
    private DcMotorEx lift;
    private DcMotorEx lift2;
    private static double speed1 = 0.475;
    private static double speed2 = speed1*2;
    private static double speed3 = speed1*1.75;
    private DcMotor swingBar;
    private CRServo claw;
    private CRServo claw2;
//    private int length = 0;
//    private int swing = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        claw = hardwareMap.get(CRServo.class, "claw");
        claw2 = hardwareMap.get(CRServo.class, "claw2");
        swingBar = hardwareMap.get(DcMotor.class, "SwingBar");

        lift2.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speed1,
                            -gamepad1.left_stick_x * speed2,
                            -gamepad1.right_stick_x * speed3
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if (gamepad2.dpad_up) {
                lift.setPower(1);
                lift2.setPower(1);
            } else if (gamepad2.dpad_down) {
                lift.setPower(-0.8);
                lift2.setPower(-0.8);
            }else {
                lift.setPower(0.1);
                lift2.setPower(0.1);
            }

//            if (gamepad2.dpad_up) {
//                switch(length) {
//                    case 1:
//                        lift.setPositionPIDFCoefficients(5);
//                        lift2.setPositionPIDFCoefficients(5);
//                        length = 2;
//                    case 2:
//                        lift.setPositionPIDFCoefficients(10);
//                        lift2.setPositionPIDFCoefficients(10);
//                        length = 3;
//                    case 3:
//                        lift.setPositionPIDFCoefficients(15);
//                        lift2.setPositionPIDFCoefficients(15);
//                }
//            } else if (gamepad2.dpad_down) {
//                switch(length) {
//                    case 3:
//                        lift.setPositionPIDFCoefficients(15);
//                        lift2.setPositionPIDFCoefficients(15);
//                        length = 3;
//                    case 2:
//                        lift.setPositionPIDFCoefficients(10);
//                        lift2.setPositionPIDFCoefficients(10);
//                        length = 1;
//                    case 1:
//                        lift.setPositionPIDFCoefficients(5);
//                        lift2.setPositionPIDFCoefficients(5);
//                }
//            }else {
//                lift.setPower(0.1);
//                lift2.setPower(0.1);
//            }

            if (gamepad2.left_bumper) {
                claw.setPower(0.5);
//                claw2.setPower(0);
            } else if (gamepad2.right_bumper) {
                claw.setPower(-0.5);
//                claw2.setPower(0);
//            } else {
//                claw.setPower(0.5);
//                claw2.setPower(0);
            }
            if (gamepad2.a) {
                swingBar.setPower(0.5);
            } else if (gamepad2.x) {
                swingBar.setPower(-0.5);
            } else {
                swingBar.setPower(0.1);
            }
        }
    }
}
