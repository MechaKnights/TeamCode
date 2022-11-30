package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Webcam Test")
public class WebcamTest extends LinearOpMode {

    SignalPipeline SignalPipeline = new SignalPipeline();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    org.firstinspires.ftc.teamcode.SignalPipeline pipeline;
    org.firstinspires.ftc.teamcode.SignalPipeline.TeamShippingElementPosition snapshotAnalysis = org.firstinspires.ftc.teamcode.SignalPipeline.TeamShippingElementPosition.RIGHT;


    private DcMotorEx lift;
    private DcMotorEx lift2;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("webcam", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        SignalPipeline = new SignalPipeline();
        camera.setPipeline(SignalPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("Realtime analysis", SignalPipeline.getAnalysis());
            telemetry.update();

            sleep(50);
        }

        waitForStart();
        
        snapshotAnalysis = SignalPipeline.getAnalysis();

        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();
        
        while (opModeIsActive()) {
            sleep(50);
            switch (snapshotAnalysis) {
                case LEFT: {
                    lift.setPower(0.5);
                    lift2.setPower(0.5);
                    sleep(800);
                    lift.setPower(0);
                    lift2.setPower(0);
                }
                case CENTER: {
                    lift.setPower(0.5);
                    lift2.setPower(0.5);
                    sleep(600);
                    lift.setPower(0);
                    lift2.setPower(0);

                }
                case RIGHT: {
                    lift.setPower(0.5);
                    lift2.setPower(0.5);
                    sleep(400);
                    lift.setPower(0);
                    lift2.setPower(0);

                }
            }
        }
    }
}
