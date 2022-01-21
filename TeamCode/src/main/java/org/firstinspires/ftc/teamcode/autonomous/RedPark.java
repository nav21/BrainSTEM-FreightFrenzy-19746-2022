package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Depositor;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "auto")
public class RedPark extends LinearOpMode {
    public static double DISTANCE = 24; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
robot.depositor.setDepositorLiftMotorEncoders();
        robot.initAuto();
        Pose2d startPose = new Pose2d(-12, -65.5, Math.toRadians(0));
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .forward(DISTANCE*0.93)
                .build();
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end().plus(new Pose2d(0, 0, Math.toRadians(8-9))))
                .forward(DISTANCE*0.6)
                .build();



        waitForStart();

        if (isStopRequested()) return;
        robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
        robot.drive.followTrajectory(trajectory1);
        robot.drive.turn(Math.toRadians(-89));
        robot.drive.followTrajectory(trajectory2);
        sleep(500);
        robot.depositor.setLiftPower(0.7);
        sleep(800);
        robot.depositor.setLiftPower(-0.7);
        sleep(800);
        robot.depositor.setLiftPower(0);




      //  robot.depositor.setLiftPower();
//        robot.drive.turn(20);
//        robot.collector.setCollectorPower(0.85);
//        sleep(3000);


        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive());
    }
}
