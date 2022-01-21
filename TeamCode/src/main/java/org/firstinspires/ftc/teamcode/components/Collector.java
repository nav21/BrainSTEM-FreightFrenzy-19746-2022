package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.autonomous.enums.CollectorPosition;
import org.firstinspires.ftc.teamcode.utils.Component;

public class Collector implements Component {
    private final DcMotor collectMotor;
    private final ServoImplEx leftCollectorFlipOutServo;
    private final ServoImplEx rightCollectorFlipOutServo;
    private final ServoImplEx collectorGateServo;

    public Collector(HardwareMap map) {
        collectMotor = map.dcMotor.get("collectorMotor");
        leftCollectorFlipOutServo = map.get(ServoImplEx.class, "leftCollectorFlipOutServo");
        rightCollectorFlipOutServo = map.get(ServoImplEx.class, "rightCollectorFlipOutServo");
        collectorGateServo = map.get(ServoImplEx.class,"collectorGateServo");
        collectMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        collectMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void initAuto() {
        //setFlipServosPosition(CollectorPosition.IN);
//        setCollectorGateServoPosition(CollectorPosition.OUT);
    }

	@Override
	public void initTeleOp() {
		//setFlipServosPosition(CollectorPosition.OUT);
//        setCollectorGateServoPosition(CollectorPosition.OUT);
	}


    @Override
    public void update() {

    }

    public String test() {
        String failures = "";

        return failures;
    }

    public void setCollectorPower(double power) {
        collectMotor.setPower(power);
    }

    //TODO: FIX THESE VALUES, I MADE THESE RANDOM
    public void setFlipServosPosition(CollectorPosition position) {
        switch (position) {
            case IN:
                leftCollectorFlipOutServo.setPosition(1);
                rightCollectorFlipOutServo.setPosition(0);
                break;
            case OUT:
                leftCollectorFlipOutServo.setPosition(0.29);
                rightCollectorFlipOutServo.setPosition(0.71);
                break;
            case TILTED_UP:
                leftCollectorFlipOutServo.setPosition(0.33);
                rightCollectorFlipOutServo.setPosition(0.67);
                break;
        }
    }
//    public void setCollectorGateServoPosition(CollectorPosition position) {
//        switch (position) {
//            case IN:
//                collectorGateServo.setPosition(0.2);
//                break;
//            case OUT:
//                collectorGateServo.setPosition(0.55);
//                break;
//        }
//    }
    public void disableFlipServos() {
        leftCollectorFlipOutServo.setPwmDisable();
        rightCollectorFlipOutServo.setPwmDisable();
    }

    public void enableFlipServos() {
        leftCollectorFlipOutServo.setPwmEnable();
        rightCollectorFlipOutServo.setPwmEnable();
    }
}
