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
    public double cpsMultiplier; // Motor Counts per second that is equivalent to 1 inch /second wheel speed.
    public double motorPowerMultiplier; // Motor power required to move the wheel at a speed of 1 in/sec
    public static final double inchesPerFoot = 12.0;

    public PowerTrain(Wheel wheel, double gearRatio, DcMotor motor, double CPR,
                      double maxSpeed, double stallTorque, double outputPower,
                      double efficiency) {
        this.wheel = wheel;
        this.gearRatio = gearRatio;
        this.motor = motor;
        this.motorEncoderCPR = CPR;
        this.motorSpeedMax = maxSpeed;
        //wheelSpeedMax should be in inches/sec
        this.wheelSpeedMax = (this.motorSpeedMax * this.gearRatio * this.wheel.circumference) / 60;
        this.motorStallTorque = stallTorque;
        this.motorOutputPower = outputPower;
        this.efficiency = efficiency;

        // Calculate the multiplier to convert 1 inch / sec wheel speed
        // into motor counts per second.
        // 1. Calculate the motor rotations per second, assuming 100% efficiency;
        //    i.e. direct-connected motor.
        double motorRPS = (1 / wheel.circumference) / gearRatio;
        // 2. Convert the motorRPS into motorCPS, still assuming 100% efficiency.
        double motorCPS = motorRPS * motorEncoderCPR;
        // 3. Adjust the above value for the inefficiency in the power train.
        //    E.g. Sprocket-chain mechanism has 95% efficiency.
        this.cpsMultiplier = motorCPS / efficiency;

        // Calculate the multiplier to convert 1 inch / sec wheel speed
        // into motor power.
        this.motorPowerMultiplier = this.cpsMultiplier / (motorEncoderCPR * motorSpeedMax / 60);
    }

    public double countsToDistance(long counts) {
        return ((counts / this.motorEncoderCPR) * this.wheel.circumference);
    }

}
