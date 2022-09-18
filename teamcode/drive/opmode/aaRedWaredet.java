package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class aaRedWaredet extends LinearOpMode {
    OpenCvWebcam webcam;
    org.firstinspires.ftc.teamcode.drive.opmode.TeamShippingElementPipeline pipeline;
    org.firstinspires.ftc.teamcode.drive.opmode.TeamShippingElementPipeline.TeamShippingElementPosition snapshotAnalysis = org.firstinspires.ftc.teamcode.drive.opmode.TeamShippingElementPipeline.TeamShippingElementPosition.RIGHT;

    private DistanceSensor H;
    private DcMotorEx lift;
    private DcMotorEx leftin;
    private DcMotorEx rightin;
    private DcMotorEx duckturn;
    private Servo push;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new org.firstinspires.ftc.teamcode.drive.opmode.TeamShippingElementPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        H = hardwareMap.get(DistanceSensor.class, "H");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        leftin = hardwareMap.get(DcMotorEx.class, "leftin");
        rightin = hardwareMap.get(DcMotorEx.class, "rightin");
        duckturn = hardwareMap.get(DcMotorEx.class, "duckturn");
        push = hardwareMap.get(Servo.class, "push");
        telemetry.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("H", String.format("%.01f in", H.getDistance(DistanceUnit.INCH)));
        telemetry.update();


        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(20, 15), Math.toRadians(0))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(2, 8, Math.toRadians(90)))
                .strafeLeft(6)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-6, 8, Math.toRadians(90)))
                .back(40)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(15, 8, Math.toRadians(-90)))
                .splineTo(new Vector2d(-15, 17), Math.toRadians(135))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(2, 8, Math.toRadians(-90)))
                .strafeLeft(12)
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(new Pose2d(10, 8, Math.toRadians(-90)))
                .forward(45)
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(new Pose2d(15, 8, Math.toRadians(-90)))
                .splineTo(new Vector2d(-15, 17), Math.toRadians(135))
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(new Pose2d(10, 8, Math.toRadians(-90)))
                .strafeLeft(11)
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(new Pose2d(20, 8, Math.toRadians(-90)))
                .forward(30)
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();

        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();


        while (opModeIsActive()) {
            sleep(50);
            leftin.setPower(-0.05);
            rightin.setPower(0.05);
            lift.setPower(.1);
            push.setPosition(1);


            switch (snapshotAnalysis) {
                case LEFT: {
                    //Insert code for lift to bottom

                    lift.setPower(1);

                    sleep(200);

                    lift.setPower(.25);


                    drive.followTrajectory(traj2);

                    leftin.setPower(0.5);
                    rightin.setPower(-0.5);

                    sleep(200);

                    leftin.setPower(0);
                    rightin.setPower(0);

                    sleep(10);
                    lift.setPower(1);
                    sleep(1000);
                    lift.setPower(0);



                    break;
                }

                case RIGHT: {
                    //Insert code for lift to top


                    lift.setPower(1);

                    sleep(1000);

                    lift.setPower(.25);
                    drive.turn(Math.toRadians(-90));
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(traj2);

                    leftin.setPower(0.7);
                    rightin.setPower(-0.7);

                    sleep(200);

                    leftin.setPower(0);
                    rightin.setPower(0);
                    drive.turn(Math.toRadians(90));
                    drive.turn(Math.toRadians(90));

                    break;
                }

                case CENTER: {
                    //code for center

                    lift.setPower(1);

                    sleep(350);

                    lift.setPower(.27);

                    drive.followTrajectory(traj2);


                    leftin.setPower(0.5);
                    rightin.setPower(-0.5);

                    sleep(200);

                    leftin.setPower(0);
                    rightin.setPower(0);



                    sleep(10);
                    lift.setPower(1);
                    sleep(1000);
                    lift.setPower(0);



                    break;
                }

            }

            drive.followTrajectory(
                    drive.trajectoryBuilder(traj2.end(), true)
                            .splineTo(new Vector2d(10, 8), Math.toRadians(-90))
                            .build()
            );
            lift.setPower(0);

            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
            sleep(200000);
        }
    }
}

