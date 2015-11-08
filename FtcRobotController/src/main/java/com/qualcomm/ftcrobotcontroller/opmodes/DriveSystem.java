package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by pranavb on 11/8/15.
 */
public class DriveSystem extends OpMode {
    DcMotor frontLeft = hardwareMap.dcMotor.get("fMotorL");
    DcMotor frontRight = hardwareMap.dcMotor.get("fMotorR");
    DcMotor rearLeft = hardwareMap.dcMotor.get("rMotorL");
    DcMotor rearRight = hardwareMap.dcMotor.get("rMotorR");

    public class Wheel {
        public class MecanumWheel{
            final double diameter = 4;
            final double frictionCoefficientMat = 1;
            final double frictionCoefficientMountain = 1;
        }
    }

    public void mecanumWheelDrive(float strafeDirection, float strafeThrottle, float turnDirection, float turnThrottle) {
        strafeDirection = Range.clip(strafeDirection, -1, 1);
        strafeThrottle = Range.clip(strafeThrottle, -1, 1);
        turnThrottle = Range.clip(turnThrottle, -1, 1);
        turnDirection = Range.clip(turnDirection, -1, 1);

        float frontLeftPwr = -(Range.clip(strafeThrottle + strafeDirection, -1, 1) - Range.clip(turnThrottle + turnDirection, -1, 1));
        float frontRightPwr = -(Range.clip(strafeDirection - strafeThrottle, -1, 1) + Range.clip(turnThrottle - turnDirection, -1, 1));
        float rearLeftPwr = Range.clip(strafeDirection - strafeThrottle, -1, 1) + Range.clip(turnThrottle + turnDirection, -1, 1);
        float rearRightPwr = Range.clip(strafeThrottle + strafeDirection, -1, 1) - Range.clip(turnThrottle - turnDirection, -1, 1);

        frontLeft.setPower(Range.clip(frontLeftPwr, -1, 1));
        rearRight.setPower(Range.clip(rearRightPwr, -1, 1));
        frontRight.setPower(Range.clip(frontRightPwr, -1, 1));
        rearLeft.setPower(Range.clip(rearLeftPwr, -1, 1));
    }

    @Override
    public void init(){

    }

    @Override
    public void loop(){

    }
}
