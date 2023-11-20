package org.firstinspires.ftc.teamcode.drive.opmode.coding;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class BlueRight extends LinearOpMode {
    OpenCvWebcam webcam;
    org.firstinspires.ftc.teamcode.drive.opmode.coding.TeamShippingElementPipeline pipeline;
    org.firstinspires.ftc.teamcode.drive.opmode.coding.TeamShippingElementPipeline.TeamShippingElementPosition snapshotAnalysis = org.firstinspires.ftc.teamcode.drive.opmode.coding.TeamShippingElementPipeline.TeamShippingElementPosition.RIGHT;

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

        pipeline = new org.firstinspires.ftc.teamcode.drive.opmode.coding.TeamShippingElementPipeline();
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

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(3)
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
            switch (snapshotAnalysis) {
                case LEFT: {
                }

                case RIGHT: {
                }

                case CENTER: {
                }
            }
        }


//PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

//    void tagToTelemetry(AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        //        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}