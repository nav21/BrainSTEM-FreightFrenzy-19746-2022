package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.enums.GatePosition;
import org.firstinspires.ftc.teamcode.buttons.ToggleButton;
import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.components.Depositor;

@TeleOp
public class BrainSTEMTeleOp extends LinearOpMode {

    private double leftTuningFactor = 1;
    private double rightTuningFactor = 1;
    private double driveInterpolationFactor = 1.5;

    private ToggleButton collectorToggle = new ToggleButton();
        private ToggleButton carouselToggle = new ToggleButton();
    //private ToggleButton collectorGateToggle = new ToggleButton();
    private ToggleButton depositorGateToggle = new ToggleButton();
    private ToggleButton extensionLiftToggle = new ToggleButton();

    private boolean lastMovedDepositorUp = false;
    private Depositor.Goal lastGoal = Depositor.Goal.IN;

    private static final double THRESHOLD = 0.001;

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
        D-pad Left:
        D-pad Down: Carousel left wheel move , click again to spin other way
        D-pad Right:
        Start:
        X: Press X once to close the gate servo, extend the depositor out, and deposit the element.
        Press X again to close the deposit servo, retract depositor, and open gate servo.
        B: Press B once to close the depositor gate servo, and press B once more to open it
        Y:
        A: Press A once to put collector gate servo in and click A again to put it out.
        Left Bumper: Reverse collector when holding
        Left Trigger: Move lift down
        Right Bumper: Toggle collector on/off
        Right Trigger: Move lift up
     */
    private double driveLeft;
    private double driveRight;
    private boolean toggleCollectorOn;
    private boolean collectorReverse;
    private double moveLiftUp;
    private double moveLiftDown;
    private boolean toggleExtension;
    private boolean carouselReverse;
    private boolean toggleCarouselOn;




    private void mapControls() {
        driveLeft = Math.pow(Math.abs(gamepad1.left_stick_y), driveInterpolationFactor) * Math.signum(-gamepad1.left_stick_y);
        driveRight = Math.pow(Math.abs(gamepad1.right_stick_y), driveInterpolationFactor) * Math.signum(-gamepad1.right_stick_y);
        toggleCollectorOn = collectorToggle.update(gamepad1.right_bumper);
        toggleCarouselOn = carouselToggle.update(gamepad1.dpad_down);
        collectorReverse = gamepad1.left_bumper;
        carouselReverse = gamepad1.dpad_up;
        moveLiftUp = gamepad1.right_trigger;
        moveLiftDown = gamepad1.left_trigger;
        toggleExtension = extensionLiftToggle.update(gamepad1.x);


    }

    @Override
    public void runOpMode() {
        // Initialize a new robot object
        BrainSTEMRobot robot = new BrainSTEMRobot(this);

        robot.initTeleOp();

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
            if ((Math.abs(driveLeft) > THRESHOLD) || (Math.abs(driveRight) > THRESHOLD)) {
                robot.drive.setMotorPowers(driveLeft, driveRight);
            } else {
                robot.drive.setMotorPowers(0, 0);
            }

            if (collectorReverse) {
                robot.collector.setCollectorPower(-1);
            } else if (toggleCollectorOn) {
                robot.collector.setCollectorPower(1);
            } else {
                robot.collector.setCollectorPower(0);
            }

            if (carouselReverse) {
                robot.collector.setCollectorPower(-0.73);
            } else if (toggleCarouselOn) {
                robot.collector.setCollectorPower(0.73);
            } else {
                robot.collector.setCollectorPower(0);
            }

//            if (gateServoToggle)
//            {
//                //robot.collector.enableFlipServos();
//                robot.collector.setFlipServosPosition(CollectorPosition.TILTED_UP);
//                robot.collector.setCollectorGateServoPosition(CollectorPosition.OUT);
//            }

//            if (depositorGateServoToggle) {
//                robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
//                robot.depositor.setGateServoPosition(GatePosition.DOWN);
//            } else {
//                robot.depositor.setGoal(Depositor.Goal.OPEN_LOOP);
//                robot.depositor.setGateServoPosition(GatePosition.UP);
//            }
//
            if (moveLiftUp > THRESHOLD) {
                robot.depositor.setLiftPower(moveLiftUp);
                robot.depositor.setGateServoPosition(GatePosition.DOWN);
                lastMovedDepositorUp = true;
            } else if (moveLiftDown > THRESHOLD) {
                robot.depositor.setLiftPower(-moveLiftDown);
                robot.depositor.setGateServoPosition(GatePosition.UP);
                lastMovedDepositorUp = false;
            } else if (lastMovedDepositorUp) {
                robot.depositor.holdLift();
            } else {
                robot.depositor.setLiftPower(0);
            }



            if (toggleExtension) {
                robot.depositor.setGoal(Depositor.Goal.OUT);
                lastGoal = Depositor.Goal.OUT;
            } else {
                robot.depositor.setGoal(Depositor.Goal.IN);
                lastGoal = Depositor.Goal.IN;
            }
            robot.depositor.update();
            telemetry.addData("lastmoveddepositorup", lastMovedDepositorUp);
            telemetry.addData("left", driveLeft);
            telemetry.addData("liftpower", moveLiftUp);
            telemetry.addData("liftpower2", -moveLiftDown);
            telemetry.addData("right", driveRight);
            telemetry.update();
        }
    }
}