package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.buttons.StickyButton;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;

@TeleOp
public class ServoGateTest extends LinearOpMode {
    private StickyButton backward = new StickyButton();
    private StickyButton forward = new StickyButton();



    ////////////
    //DRIVER 1//
    ////////////
    /*
        NEW DRIVER 1 CONTROLS
        Left Stick X:
        Left Stick Y: Drive left side
        Left Stick Button: Press left button once to enable collector flip servos and move it to exactly horizontal, press again to disable flip servos
        Right Stick X:
        Right Stick Y: Drive right side
        Right Stick Button:
        D-pad Up:
        D-pad Down:
        D-pad Left:
        D-pad Right:
        Start:
        X: Press X once to close the gate servo, extend the depositor out, and deposit the element.
        Press X again to close the deposit servo, retract depositor, and open gate servo.
        B:
        Y:
        A: Press A once to put collector gate servo in and click A again to put it out.
        Left Bumper: Reverse collector when holding
        Left Trigger: Move lift down
        Right Bumper: Toggle collector on/off
        Right Trigger: Move lift up
     */
    private boolean servoGateFoward;
    private boolean servoGateBackward;
private static double position = 0.2;


    private void mapControls() {
       forward.update(gamepad1.x);
       backward.update(gamepad1.a);

       servoGateBackward = forward.getState();
       servoGateFoward = backward.getState();

    }

    @Override
    public void runOpMode() {
        // Initialize a new robot object
        BrainSTEMRobot robot = new BrainSTEMRobot(this);

        while (!opModeIsActive() && !isStopRequested()) {
            //Status to show if telemetry was initialized
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }


        while (opModeIsActive()) {
            mapControls();

            robot.collector.disableFlipServos();

            //If the x value of the left stick, the y value of the left stick, or the x value of
            //the right stick is greater than the THRESHOLD value, move the robot

            if (servoGateBackward) {
                robot.depositor.setReleaseGateServoPosition(position -= 0.05);
                telemetry.addData("test", "testting");
                telemetry.update();
            }
            if (servoGateFoward) {
                robot.depositor.setReleaseGateServoPosition(position += 0.05);
                telemetry.addData("test", "testt");
                telemetry.update();
            }
            telemetry.addData("Servo Position:", position);
            telemetry.update();
        }
    }
}