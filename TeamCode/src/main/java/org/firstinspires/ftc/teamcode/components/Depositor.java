package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
import org.firstinspires.ftc.teamcode.autonomous.enums.DepositorPosition;
import org.firstinspires.ftc.teamcode.autonomous.enums.GatePosition;
import org.firstinspires.ftc.teamcode.utils.Component;

/**
 * Created by parvs on 11/9/2018.
 */

public class Depositor implements Component {
    public enum Goal {
        OPEN_LOOP, OUT, IN, DEPOSIT
    }

    private static final double HOLD_LIFT_POWER = 0.1;

    private final DcMotor depositorLiftMotor;
    private final Servo extendDepositorServo;
    private final Servo gateServo;
    private final Servo releaseElementServo;
    //private final ServoImplEx gateServo; u

    //TODO: adjust these values maybe
    private TimerCanceller closeGateServoCanceller = new TimerCanceller(500);
    private TimerCanceller retractReleaseElementServoCanceller = new TimerCanceller(800);
    private TimerCanceller extensionServoOutCanceller = new TimerCanceller(500);

    private Goal goal = Goal.IN;

    public Depositor(HardwareMap map) {
        depositorLiftMotor = map.dcMotor.get("depositorLiftMotor");
        extendDepositorServo = map.servo.get("extendDepositorServo");
        gateServo = map.servo.get("gateServo");
        releaseElementServo = map.servo.get("releaseElementServo");

        depositorLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        depositorLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        depositorLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void initAuto() {
//        setExtendDepositorServoPosition(DepositorPosition.IN);
//        setGateServoPosition(GatePosition.DOWN);
//        setReleaseElementServoPosition(DepositorPosition.CLOSED);
    }

    @Override
    public void initTeleOp() {
//        initAuto();
//        setGateServoPosition(GatePosition.DOWN);
    }

    @Override
    public void update() {
        switch (goal) {
            case OPEN_LOOP:
                break;
            case OUT:
                if (extensionServoOutCanceller.isConditionMet()) {
                    setReleaseElementServoPosition(DepositorPosition.OPEN);
                }
                setExtendDepositorServoPosition(DepositorPosition.OUT);
                break;
            case IN:
                if (retractReleaseElementServoCanceller.isConditionMet()) {
                    setExtendDepositorServoPosition(DepositorPosition.IN);
                }
                setReleaseElementServoPosition(DepositorPosition.CLOSED);
                //setGateServoPosition(DepositorPosition.OPEN);
                break;
        }
    }

    public void setGoal(Goal goal) {
        if (this.goal != goal) {
            closeGateServoCanceller.reset();
            extensionServoOutCanceller.reset();
            retractReleaseElementServoCanceller.reset();
            this.goal = goal;
        }
    }

    public Goal getGoal() {
        return goal;
    }

    public String test() {
        String failures = "";

        return failures;
    }

    public void setLiftPower(double power) {
        depositorLiftMotor.setPower(power);
    }

    //TODO: FIX THESE VALUES, I MADE THESE RANDOM
    public void holdLift() {
        depositorLiftMotor.setPower(HOLD_LIFT_POWER);
    }


    //TODO: FIX THESE VALUES, I MADE THESE RANDOM
    public void setExtendDepositorServoPosition(DepositorPosition position) {
        switch (position) {
            case IN:
                extendDepositorServo.setPosition(0.99);
                break;
            case OUT:
                extendDepositorServo.setPosition(0.2);
                break;
        }
    }


    public void setReleaseGateServoPosition(double position) {
        releaseElementServo.setPosition(position);
    }


    public void setExtensionServoPosition(double position) {
 extendDepositorServo.setPosition(position);
    }

    //TODO: FIX THESE VALUES, I MADE THESE RANDOM
    public void setGateServoPosition(GatePosition position) {
        switch (position) {
            case DOWN:
                gateServo.setPosition(0.01);
                break;
            case UP:
                gateServo.setPosition(0.51);
                break;
        }
    }

    public void setDepositorLiftMotorEncoders () {
        depositorLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //TODO: FIX THESE VALUES, I MADE THESE RANDOM
    public void setReleaseElementServoPosition(DepositorPosition position) {
        switch (position) {
            case CLOSED:
                releaseElementServo.setPosition(0);
                break;
            case OPEN:
                releaseElementServo.setPosition(0.3);
                break;
        }
    }
}
