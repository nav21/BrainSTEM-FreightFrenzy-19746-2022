package org.firstinspires.ftc.teamcode.tests.necessary;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.SixWheelTankDrive;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 */
@Config
@TeleOp(group = "drive")
public class MotorDirectionDebugger extends LinearOpMode {
    public static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SixWheelTankDrive drive = new SixWheelTankDrive(hardwareMap);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (gamepad1.x) {
                drive.setMotorPowers(MOTOR_POWER, 0, 0, 0, 0, 0);
                telemetry.addLine("Running Motor: Left Front");
            } else if (gamepad1.y) {
                drive.setMotorPowers(0, MOTOR_POWER, 0, 0, 0, 0);
                telemetry.addLine("Running Motor: Left Middle");
            } else if (gamepad1.b) {
                drive.setMotorPowers(0, 0, MOTOR_POWER, 0, 0, 0);
                telemetry.addLine("Running Motor: Left Rear");
            } else if (gamepad1.a) {
                drive.setMotorPowers(0, 0, 0, MOTOR_POWER, 0, 0);
                telemetry.addLine("Running Motor: Right Front");
            } else if (gamepad1.dpad_left) {
                drive.setMotorPowers(0, 0, 0, 0, MOTOR_POWER, 0);
                telemetry.addLine("Running Motor: Right Middle");
            } else if (gamepad1.dpad_right) {
                drive.setMotorPowers(0, 0, 0, 0, 0, MOTOR_POWER);
                telemetry.addLine("Running Motor: Right Rear");
            } else {
                drive.setMotorPowers(0, 0, 0, 0, 0, 0);
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
