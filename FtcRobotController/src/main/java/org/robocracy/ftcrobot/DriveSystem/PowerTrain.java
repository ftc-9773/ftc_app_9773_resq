package org.robocracy.ftcrobot.DriveSystem;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robocracy on 11/17/2015.
 */
public class PowerTrain {
    Wheel wheel;
    double gearRatio;
    DcMotor motor;
    double motorEncoderCPR; // = 1120 for AndyMark Neverest 40
    double motorSpeedMax; // = 160 rpm for AndyMark Neverest 40
    double motorStallTorque; // = 350 oz-in for AndyMark Neverest 40
    double motorOutputPower; // = 14 Watts for AndyMark Neverest 40
    double efficiency; // = 0.95 for sprocket-chain and 100% for direct connected motors
    double wheelSpeedMax; // in inches/sec
    static final double inchesPerFoot = 12.0;

    public PowerTrain(Wheel wheel, double gearRatio, DcMotor motor, double CPR,
                      double maxSpeed, double stallTorque, double outputPower,
                      double efficiency) {
        this.wheel = wheel;
        this.gearRatio = gearRatio;
        this.motor = motor;
        this.motorEncoderCPR = CPR;
        this.motorSpeedMax = maxSpeed;
        this.wheelSpeedMax = this.motorSpeedMax * this.gearRatio * this.wheel.circumference;
        this.motorStallTorque = stallTorque;
        this.motorOutputPower = outputPower;
        this.efficiency = efficiency;
    }
}
