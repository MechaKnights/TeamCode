package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private DcMotorEx intake;
    private DcMotorEx lift2;
    private CRServo extendLift;
    private static double speed1 = 0.475;
    private static double speed2 = speed1*2;
    private DistanceSensor sensor;
    private ColorSensor colors;
    String intakeSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        extendLift = hardwareMap.get(CRServo.class, "extendLift");
        sensor = hardwareMap.get(DistanceSensor.class, "sensor");
        colors = hardwareMap.get(ColorSensor.class, "colors");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);
        intakeSensor = "false";
        waitForStart();
        while (!isStopRequested()) {
            double speed2 = speed1*2;
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speed1,
                            -gamepad1.left_stick_x * speed2,
                            -gamepad1.right_stick_x * speed1
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("sensor", String.format("%.01f in", sensor.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Intake:", String.format("", intakeSensor.getClass()));
            telemetry.update();

            if (gamepad2.dpad_up && (sensor.getDistance(DistanceUnit.INCH)<3)) {
                while (sensor.getDistance(DistanceUnit.INCH)!= 3) {
                    lift.setPower(0.5);
                    lift2.setPower(0.5);
                }
                lift.setPower(0);
                lift2.setPower(0);
            } else if (gamepad2.dpad_down && (sensor.getDistance(DistanceUnit.INCH)>3)) {
                while (sensor.getDistance(DistanceUnit.INCH)!= 3) {
                    lift.setPower(-0.5);
                    lift2.setPower(-0.5);
                }
            }else {
                lift.setPower(0.1);
                lift2.setPower(0.1);
            }

            if (gamepad2.left_bumper) {
                intake.setPower(.75);
            } else if (gamepad2.right_bumper) {
                intake.setPower(-.75);
            } else {
                intake .setPower(0);
            }

            if (gamepad2.x) {
                extendLift.setPower(.5);
            } else if (gamepad2.y) {
                extendLift.setPower(-.5);
            }else {
                extendLift.setPower(0.075);
            }
            if (gamepad1.right_bumper) {
                speed1 = 0.25;
            } else {
                speed1 = 0.475;
            }
            if (colors.equals(colors.red()) || colors.equals(colors.blue())) {
                intakeSensor = "true";
            }
            else {
                intakeSensor = "false";
            }
        }
    }
}