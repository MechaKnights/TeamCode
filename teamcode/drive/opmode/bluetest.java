package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(group = "drive")
public class bluetest extends LinearOpMode {
    private DistanceSensor H;
    private DcMotorEx lift;
    private DcMotorEx leftin;
    private DcMotorEx rightin;
    private DcMotorEx duckturn;
    private Servo push;


    @Override
    public void runOpMode() throws InterruptedException {
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
                .strafeLeft(42)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(0, 42, Math.toRadians(0)))
                .forward(6)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(6, 42, Math.toRadians(0)))
                .back(6)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-4, 42, Math.toRadians(0)))
                .strafeRight(40)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-4, 2, Math.toRadians(0)))
                .back(20)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-24, 2, Math.toRadians(180)))
                .strafeLeft(2)
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(-24, 0, Math.toRadians(180)))
                .forward(10)
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(new Pose2d(-34, 0, Math.toRadians(180)))
                .forward(5)
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(new Pose2d(-39, 0, Math.toRadians(180)))
                .back(5)
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(new Pose2d(-29, 0, Math.toRadians(0)))
                .forward(50)
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(new Pose2d(21, 0, Math.toRadians(90)))
                .forward(15)
                .build();
        Trajectory traj12 = drive.trajectoryBuilder(new Pose2d(21, 15, Math.toRadians(90)))
                .forward(5)
                .build();
        Trajectory traj13 = drive.trajectoryBuilder(new Pose2d(21, 20, Math.toRadians(90)))
                .back(20)
                .build();
        Trajectory traj14 = drive.trajectoryBuilder(new Pose2d(21, 0, Math.toRadians(180)))
                .forward(60)
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(new Pose2d(-39, 0, Math.toRadians(180)))
                .forward(5)
                .build();
        waitForStart();
        push.setPosition(1);
        leftin.setPower(-0.05);
        rightin.setPower(0.05);
        lift.setPower(.1);

        drive.followTrajectory(traj1);
        lift.setPower(1);

        sleep(1400);

        lift.setPower(.1);
        drive.followTrajectory(traj2);

        leftin.setPower(0.5);
        rightin.setPower(-0.5);

        sleep(200);

        leftin.setPower(0);
        rightin.setPower(0);

        drive.followTrajectory(traj3);

        lift.setPower(0);

        drive.followTrajectory(traj4);

        drive.followTrajectory(traj5);

        drive.turn(Math.toRadians(180));

        drive.followTrajectory(traj6);

        leftin.setPower(-0.5);
        rightin.setPower(0.5);

        drive.followTrajectory(traj7);

        drive.followTrajectory(traj8);

        leftin.setPower(-0.05);
        rightin.setPower(0.05);

        lift.setPower(.1);

        drive.followTrajectory(traj9);

        drive.turn(Math.toRadians(-180));

        drive.followTrajectory(traj10);

        drive.turn(Math.toRadians(90));

        drive.followTrajectory(traj11);

        lift.setPower(1);

        sleep(1400);

        lift.setPower(.1);
        drive.followTrajectory(traj12);

        leftin.setPower(0.5);
        rightin.setPower(-0.5);

        sleep(200);

        leftin.setPower(0);
        rightin.setPower(0);
        lift.setPower(0);

        drive.followTrajectory(traj13);

        drive.turn(Math.toRadians(90));

        leftin.setPower(-0.5);
        rightin.setPower(0.5);

        drive.followTrajectory(traj14);

        drive.followTrajectory(traj15);
    }
}
