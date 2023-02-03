package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Red Right Auto")
public class RedR extends LinearOpMode {

    OpenCvWebcam webcam;
    SignalPipeline.TeamShippingElementPosition snapshotAnalysis = SignalPipeline.TeamShippingElementPosition.RIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SignalPipeline());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(33)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(33,0,0))
                .forward(10)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(43,0,0))
                .back(10)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(33,0,0))
                .strafeRight(55)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(33,0,0))
                .strafeLeft(55)
                .build();
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", SignalPipeline.getAnalysis());
            telemetry.update();

            sleep(50);
        }

        snapshotAnalysis = SignalPipeline.getAnalysis();

        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        while (opModeIsActive()) {
            sleep(50);
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            switch (snapshotAnalysis) {
                case LEFT: {
                    drive.followTrajectory(traj3);
                    drive.followTrajectory(traj4);
                    sleep(20000);
                }
                case CENTER: {
                    drive.followTrajectory(traj3);
                    sleep(20000);
                }
                case RIGHT: {
                    drive.followTrajectory(traj3);
                    drive.followTrajectory(traj5);
                    sleep(20000);
                }
            }
        }
    }
}