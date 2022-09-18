package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(group = "drive")
public class Teleop extends LinearOpMode {
    private DigitalChannel redLED1;
    private DigitalChannel greenLED1;
    private DigitalChannel redLED2;
    private DigitalChannel greenLED2;
    private DistanceSensor item;
    private DcMotorEx lift;
    private DcMotorEx leftin;
    private DcMotorEx rightin;
    private DcMotorEx duckturn;
    private Servo push;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        item = hardwareMap.get(DistanceSensor.class, "item");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        leftin = hardwareMap.get(DcMotorEx.class, "leftin");
        rightin = hardwareMap.get(DcMotorEx.class, "rightin");
        duckturn = hardwareMap.get(DcMotorEx.class, "duckturn");
        push = hardwareMap.get(Servo.class, "push");
        redLED1 = hardwareMap.get(DigitalChannel.class, "red1");
        greenLED1 = hardwareMap.get(DigitalChannel.class, "green1");
        redLED2 = hardwareMap.get(DigitalChannel.class, "red2");
        greenLED2 = hardwareMap.get(DigitalChannel.class, "green2");
        duckturn.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.update();


        waitForStart();
        redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED2.setMode(DigitalChannel.Mode.OUTPUT);
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*1,
                            -gamepad1.left_stick_x*1,
                            -gamepad1.right_stick_x*.75
                    )
            );

            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("item", String.format("%.01f in", item.getDistance(DistanceUnit.INCH)));
            telemetry.update();

            push.setPosition(1);


            if (item.getDistance(DistanceUnit.INCH) > 2){
                greenLED1.setState(false);
                redLED1.setState(true);
            } else {
                greenLED1.setState(true);
                redLED1.setState(false);
            }
            if (item.getDistance(DistanceUnit.INCH) > 2){
                greenLED2.setState(false);
                redLED2.setState(true);
            } else {
                greenLED2.setState(true);
                redLED2.setState(false);
            }

            if (gamepad2.dpad_up) {
                    lift.setPower(.75);
            } else if (gamepad2.dpad_down) {
                    lift.setPower(-.75);
            }else {
                lift.setPower(0.25);
            }


            if (gamepad1.right_bumper) {
                leftin.setPower(0.7);
                rightin.setPower(-0.7);
            } else if (gamepad1.left_bumper) {
                leftin.setPower(-0.4);
                rightin.setPower(0.4);
            } else {
                leftin.setPower(-0.05);
                rightin.setPower(0.05);
            }


            if (gamepad1.a) {
                duckturn.setVelocity(1300);
            } else if (gamepad1.b) {
                duckturn.setVelocity(-1300);
                }
            else if (gamepad1.dpad_left) {
                duckturn.setVelocity(1950);
            }
            else if (gamepad1.dpad_right) {
                duckturn.setVelocity(-1950);
            } else {
                duckturn.setVelocity(0);
            }

        }
    }
}

