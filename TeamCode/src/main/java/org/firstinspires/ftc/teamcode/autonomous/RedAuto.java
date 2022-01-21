package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.enums.DepositorPosition;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Depositor;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "auto")
public class RedAuto extends LinearOpMode {
    public static double DISTANCE = 24; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
robot.depositor.setDepositorLiftMotorEncoders();
        robot.initAuto();
        Pose2d startPose = new Pose2d(-36, -63.63, Math.toRadians(-90));
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .back(DISTANCE*0.875)
                .build();
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end().plus(new Pose2d(0, 0, Math.toRadians(-35))))
                .back(DISTANCE*0.58)
                .build();
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                .forward(DISTANCE*1.46)
                .build();
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end().plus(new Pose2d(0, 0, Math.toRadians(-31))))
                .forward(DISTANCE*0.4)
                .build();
        Trajectory trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end().plus(new Pose2d(0,0, Math.toRadians(-70))))
                .forward(DISTANCE)
                .build();


        waitForStart();

        if (isStopRequested()) return;
        robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
        robot.drive.followTrajectory(trajectory1);
        robot.drive.turn(Math.toRadians(-35));
        robot.drive.followTrajectory(trajectory2);
        sleep(500);
        robot.depositor.setLiftPower(0.7);
        sleep(1000);
        robot.depositor.setLiftPower(0.01);
        sleep(1000);
        robot.depositor.setExtendDepositorServoPosition(DepositorPosition.OUT);
        sleep(1000);
        robot.depositor.setReleaseElementServoPosition(DepositorPosition.OPEN);
        sleep(1000);
        robot.depositor.setReleaseElementServoPosition(DepositorPosition.CLOSED);
        sleep(1000);
        robot.depositor.setExtendDepositorServoPosition(DepositorPosition.IN);
        sleep(1000);
        robot.depositor.setLiftPower(-0.7);
        sleep(800);
        robot.depositor.setLiftPower(0);
        robot.drive.followTrajectory(trajectory3);
        robot.drive.turn(Math.toRadians(-31));
        robot.drive.followTrajectory(trajectory4);
        robot.collector.setCollectorPower(0.37);
        sleep(4000);
        robot.collector.setCollectorPower(0);
        robot.drive.turn(Math.toRadians(-70));
        robot.drive.followTrajectory(trajectory5);


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
