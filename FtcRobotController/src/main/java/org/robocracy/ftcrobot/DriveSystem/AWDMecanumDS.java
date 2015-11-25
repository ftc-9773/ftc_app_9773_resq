package org.robocracy.ftcrobot.DriveSystem;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.robocracy.ftcrobot.util.PIDController;

import java.util.InputMismatchException;
import java.util.Timer;

/**
 * Created by burugula on 11/16/2015.
 * This class implements the drive system with the following characteristics:
 * 1) The drive system has of 4 wheels.
 * 2) All the 4 wheels are mecanum wheels
 * 3) Each of the 4 mecanum wheels is connected to a separate DC motor
 * 4) Rear wheels are directly connected to their motors
 * 5) Front wheels are connected to their motors via chain and sprocket.
 */
public class AWDMecanumDS implements DriveSystemInterface {
    // Index convention for motors, wheels, and other variables:
    // 0: Front Left
    // 1: Front Right
    // 2: Rear Left
    // 3: Rear Right
    PowerTrain[] powerTrain;
    PIDController[] motorController;
    LinearOpMode curOpmode;
    double robotMaxSpeed;
    double robotLength, robotWidth;

    public AWDMecanumDS(LinearOpMode myOpmode) {
        double wheelDiameter, forwardFrictionCoeff, sidewaysFrictionCoeff;
        double[] gearRatio = new double[4];
        double[] efficiency = new double[4];

        this.curOpmode = myOpmode;

        DcMotor[] motors = new DcMotor[4];
        motors[0] = myOpmode.hardwareMap.dcMotor.get("fMotorL");
        motors[1] = myOpmode.hardwareMap.dcMotor.get("fMotorR");
        motors[2] = myOpmode.hardwareMap.dcMotor.get("rMotorL");
        motors[3] = myOpmode.hardwareMap.dcMotor.get("rMotorR");
        motors[0].setDirection(DcMotor.Direction.REVERSE); // Front Left
        motors[3].setDirection(DcMotor.Direction.REVERSE); // Rear Right
        for (int i=0; i<4; i++) {
            motors[i].setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        wheelDiameter = 4.0;
        forwardFrictionCoeff = 0.5;
        sidewaysFrictionCoeff = 0.6;
        Wheel[] wheels = new Wheel[4];
        for (int i=0; i<4; i++) {
            wheels[i] = new MecanumWheel(wheelDiameter, forwardFrictionCoeff, sidewaysFrictionCoeff);
        }

        double motorEncoderCPR = 1120; // for AndyMark Neverest 40
        double motorSpeedMax = 160; // rpm for AndyMark Neverest 40
        double motorStallTorque = 350; // oz-in for AndyMark Neverest 40
        double motorOutputPower = 14; // Watts for AndyMark Neverest 40

        for (int i=0; i<4; i++) {
            gearRatio[i] = 1.0;
        }
        //        FLgearRatio = FRgearRatio = RLgearRatio = RRgearRatio = 1.0;
        // We assume that the motor power efficiency when connected through sprocket-chain is
        // 95% of the power delivered when directly connected.
        efficiency[0] = efficiency[1] = 0.95;
        efficiency[2] = efficiency[3] = 1.0;


        this.powerTrain = new PowerTrain[4];
        for (int i=0; i<4; i++) {
            DbgLog.error(String.format("reached here : i = %d", i));
            this.powerTrain[i] = new PowerTrain(wheels[i], gearRatio[i], motors[i], motorEncoderCPR,
                    motorSpeedMax, motorStallTorque, motorOutputPower, efficiency[i]);
        }

        // Determine the maxSpeed for this robot in inches/sec
        this.robotMaxSpeed = 100.0;
        for (int i=0; i<4; i++) {
            this.robotMaxSpeed = Math.min(this.robotMaxSpeed, this.powerTrain[i].wheelSpeedMax);
        }

        this.motorController = new PIDController[4];
        for (int i=0; i<4; i++) {
            // ToDo:  Determine the corect values for Kp, Ki, and Kd and pass them in the constructor below.
            this.motorController[i] = new PIDController(0, 0, 0, 0, 0, 0);
        }
        this.robotLength = 12; // in  inches
        this.robotWidth  = 10; // in  inches
    }

    // Speed should be specified in in/sec.
    // Phase 1: No PID Controller
    //Distance should be specified in inches.
    @Override
    public void autoMove(DriveSystemInterface.RobotDirection dir, double distance, double speed)  throws InterruptedException {
        double[] distanceTravelledByWheel = new double[4];
        double[] motorPower = new double[4];
        double[] correction = new double[4];
        int[] motorPosition = new int[4];
        int[] prevPosition = new int[4];

        // Check that the parameters are not our of bounds
        if ((distance < 0.0) || (distance > (144 * Math.sqrt(2)))) {
            DbgLog.error(String.format("distance %.04f specified is invalid\n", distance));
        }

        if ((speed < 0.0) || (speed > this.robotMaxSpeed)) {
            DbgLog.error(String.format("speed %.04f specified is invalid\n", speed));

        }
        for (int i=0; i<4; i++) {
            this.motorController[i].setSetPoint(speed);
        }
        for (int i=0; i<4; i++) {
            prevPosition[i] = this.powerTrain[i].motor.getCurrentPosition();
        }


        // It is assumed that all the motors are already set to USE_ENCODER mode
        // during initialization
        long startTime = System.nanoTime();
        long endTime = 0;
        double elapsedTime = 0;
        double distanceTravelled = 0.0;

        // Determine the Power for each Motor/PowerTrain
        for (int i=0; i<4; i++) {
            motorPower[i] = speed * this.powerTrain[i].motorPowerMultiplier;
            this.powerTrain[i].motor.setPower(motorPower[i]);
        }

        //Set the power for each motor
        while (curOpmode.opModeIsActive() && (Math.abs(distanceTravelled) < distance)) {

            // Wait for one hardware cycle
            this.curOpmode.waitForNextHardwareCycle();

            // Calculate elapsed time
            endTime = System.nanoTime();
            elapsedTime = (double) (endTime - startTime) / (10^9);
            startTime = endTime;

            // Read the current encoder values
            for (int i=0; i<4; i++) {
                motorPosition[i] = this.powerTrain[i].motor.getCurrentPosition();
                distanceTravelledByWheel[i] = this.powerTrain[i].countsToDistance((motorPosition[i] - prevPosition[i]));
                correction[i] = this.motorController[i].getNextOutputValue(distanceTravelledByWheel[i] / elapsedTime);
            }

            // Calculate error in counts per second.
            // Determine the Power for each Motor/PowerTrain

            //Set the power for each motor
            for (int i=0; i<4; i++) {
                if(dir == RobotDirection.FORWARD){
                    this.powerTrain[i].motor.setPower(motorPower[i] - correction[i]);
                }
                else if(dir == RobotDirection.BACKWARD){
                    this.powerTrain[i].motor.setPower((motorPower[i] - correction[i]) * -1);
                }
                else if(dir == RobotDirection.LEFT){
                    if(i == 0 || i == 3) {
                        this.powerTrain[i].motor.setPower((motorPower[i] - correction[i]) * -1);
                    }
                    else{
                        this.powerTrain[i].motor.setPower(motorPower[i] - correction[i]);
                    }
                    DbgLog.msg(String.format("motor power %.04f", motorPower[i] - correction[i]));
                }
                else if(dir == RobotDirection.RIGHT){
                    if(i == 1 || i == 2) {
                        this.powerTrain[i].motor.setPower((motorPower[i] - correction[i]) * -1);
                    }
                    else{
                        this.powerTrain[i].motor.setPower(motorPower[i] - correction[i]);
                    }
                    DbgLog.msg(String.format("motor power %.04f", motorPower[i] - correction[i]));
                }
            }
            System.arraycopy(motorPosition, 0, prevPosition, 0, 4);
            distanceTravelled += (Math.abs(distanceTravelledByWheel[0]) +
                                  Math.abs(distanceTravelledByWheel[1]) +
                                  Math.abs(distanceTravelledByWheel[2]) +
                                  Math.abs(distanceTravelledByWheel[3])) / 4;

/*
            if(dir == RobotDirection.LEFT || dir == RobotDirection.RIGHT) {
                distanceTravelled += (-distanceTravelledByWheel[0] + distanceTravelledByWheel[1] + distanceTravelledByWheel[2] - distanceTravelledByWheel[3]) / 4;
            }
            else if(dir == RobotDirection.BACKWARD || dir == RobotDirection.FORWARD) {
                distanceTravelled += (distanceTravelledByWheel[0] + distanceTravelledByWheel[1] + distanceTravelledByWheel[2] + distanceTravelledByWheel[3]) / 4;
            }
*/
            DbgLog.msg(String.format("Distance travelled: %f", distanceTravelled));
        }

        for(int i = 0; i < 4; i++){
            this.powerTrain[i].motor.setPower(0);
        }
        // Wait for one hardware cycle for the setPower(0) to take effect.
        this.curOpmode.waitForNextHardwareCycle();
    }

    @Override
    public void autoMove(RobotDirection direction, int intensity, double speed, OpticalDistanceSensor ods) throws InterruptedException{
        if ((speed < 0.0) || (speed > this.robotMaxSpeed)) {
            DbgLog.error(String.format("speed %.04f specified is invalid\n", speed));

        }

    }

    @Override
    public void autoTurn(RobotDirection direction, double degrees, double speed) throws InterruptedException{
        for(int i = 0;i<4;i++) {
            this.powerTrain[i].motor.setPower(0.5);
            this.curOpmode.sleep(1000);
            this.powerTrain[i].motor.setPower(0);
        }
    }

    // angle is specified in degres:  -360 < angle < 360
    @Override
    public void autoMecanum(int angle, int distance, double speed, int robotSpinDegrees) throws InterruptedException {
        double[] distanceTravelledByWheel = new double[4];
        double[] motorPower = new double[4];
        int[] motorPosition = new int[4];
        int[] prevPosition = new int[4];
        double Vx, Vy, Omega;
        double[] speedOfWheel = new double[4];
        double  correction = 0.0;

        if ((angle <= -360) || (angle >= 360)) {
            DbgLog.error(String.format("Invalid angle %d\n", angle));
            return;
        }

        if (distance < 0 || distance > 144) {
            DbgLog.error(String.format("Invalid distance %d\n", distance));
            return;
        }

        if (speed <= 0 || speed >= this.robotMaxSpeed) {
            DbgLog.error(String.format("Invalid speed %f\n", speed));
            return;
        }

        if (robotSpinDegrees <= -360 || robotSpinDegrees >= 360) {
            DbgLog.error(String.format("Invalid robotSpinDegrees %f\n", robotSpinDegrees));
            return;
        }

        // Find the quadrant corresponding to the angle
        if (angle < 0) {
            angle = 360 + angle;
        }
        int quadrant = angle / 90;
        int angleInQuadrant = angle % 90;
        // If the angle is in 1st or 3rd quadrants
        if (quadrant == 0 || quadrant == 2) {
            Vx = Math.cos(Math.toRadians((double)angleInQuadrant)) * speed;
            Vy = Math.sin(Math.toRadians((double)angleInQuadrant)) * speed;
        }
        else {
            Vy = Math.cos(Math.toRadians((double)angleInQuadrant)) * speed;
            Vx = Math.sin(Math.toRadians((double)angleInQuadrant)) * speed;
        }

        // Convert robotSpinDegrees into angular velocity
        Omega = Math.toRadians(robotSpinDegrees) / (speed / distance);

        //Calculate the speed of each wheel
        // Reference: Omni Drive system presentation by Andy Baker and Ian Mackenzie
        speedOfWheel[0] = Vy + Vx - (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[1] = Vy - Vx + (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[2] = Vy - Vx - (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[3] = Vy + Vx + (Omega * (this.robotWidth + this.robotLength) / 2);

        for (int i=0; i<4; i++) {
            this.motorController[i].setSetPoint(speedOfWheel[i]);
        }
        for (int i=0; i<4; i++) {
            prevPosition[i] = this.powerTrain[i].motor.getCurrentPosition();
        }
        // It is assumed that all the motors are already set to USE_ENCODER mode
        // during initialization
        long startTime = System.nanoTime();
        long endTime = 0;
        double elapsedTime = 0;
        double distanceTravelled = 0.0;

        // Determine the Power for each Motor/PowerTrain
        for (int i=0; i<4; i++) {
            motorPower[i] = speedOfWheel[i] * this.powerTrain[i].motorPowerMultiplier;
            this.powerTrain[i].motor.setPower(motorPower[i]);
        }

        //Set the power for each motor
        while (curOpmode.opModeIsActive() && (Math.abs(distanceTravelled) < distance)) {

            // Wait for one hardware cycle
            this.curOpmode.waitForNextHardwareCycle();

            // Calculate elapsed time
            endTime = System.nanoTime();
            elapsedTime = (double) (endTime - startTime) / (10 ^ 9);
            startTime = endTime;

            // Read the current encoder values
            for (int i = 0; i < 4; i++) {
                motorPosition[i] = this.powerTrain[i].motor.getCurrentPosition();
                distanceTravelledByWheel[i] = this.powerTrain[i].countsToDistance((motorPosition[i] - prevPosition[i]));
                correction = this.motorController[i].getNextOutputValue(distanceTravelledByWheel[i] / elapsedTime);
                this.powerTrain[i].motor.setPower(motorPower[i] - correction);
            }
            System.arraycopy(motorPosition, 0, prevPosition, 0, 4);
            distanceTravelled += (Math.abs(distanceTravelledByWheel[0]) +
                    Math.abs(distanceTravelledByWheel[1]) +
                    Math.abs(distanceTravelledByWheel[2]) +
                    Math.abs(distanceTravelledByWheel[3])) / 4;

            DbgLog.msg(String.format("Distance travelled: %f", distanceTravelled));
        }

        for(int i = 0; i < 4; i++){
            this.powerTrain[i].motor.setPower(0);
        }

        // Wait for one hardware cycle for the setPower(0) to take effect.
        this.curOpmode.waitForNextHardwareCycle();
    }

    @Override
    public void driveMecanum(int angle, double speedMultiplier, double Omega) throws  InterruptedException{
        double[] speedOfWheel = new double[4];
        double[] motorPower = new double[4];
        double Vx, Vy;

        if ((angle <= -360) || (angle >= 360)) {
            DbgLog.error(String.format("Invalid angle %d\n", angle));
            return;
        }

        if (speedMultiplier < 0 || speedMultiplier > 1) {
            DbgLog.error(String.format("Invalid speedMultiplier %f\n", speedMultiplier));
            return;
        }

        // Find the quadrant corresponding to the angle
        if (angle < 0) {
            angle = 360 + angle;
        }
        int quadrant = angle / 90;
        int angleInQuadrant = angle % 90;
        double speed = speedMultiplier * this.robotMaxSpeed;

        // If the angle is in 1st or 3rd quadrants
        if (quadrant == 0 || quadrant == 2) {
            Vx = Math.cos(Math.toRadians((double)angleInQuadrant)) * speed;
            Vy = Math.sin(Math.toRadians((double)angleInQuadrant)) * speed;
        }
        else {
            Vy = Math.cos(Math.toRadians((double)angleInQuadrant)) * speed;
            Vx = Math.sin(Math.toRadians((double)angleInQuadrant)) * speed;
        }

        //Calculate the speed of each wheel
        // Reference: Omni Drive system presentation by Andy Baker and Ian Mackenzie
        speedOfWheel[0] = Vy + Vx - (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[1] = Vy - Vx + (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[2] = Vy - Vx - (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[3] = Vy + Vx + (Omega * (this.robotWidth + this.robotLength) / 2);

        // Determine the Power for each Motor/PowerTrain
        for (int i=0; i<4; i++) {
            motorPower[i] = speedOfWheel[i] * this.powerTrain[i].motorPowerMultiplier;
            this.powerTrain[i].motor.setPower(motorPower[i]);
        }

        // Wait for one hardware cycle for the setPower(0) to take effect.
        this.curOpmode.waitForNextHardwareCycle();
    }

    @Override
    public void driverMove(RobotDirection dir, double speed) {

    }

    @Override
    public void driverTurn(RobotDirection direction, double speed) {

    }

    @Override
    public void mecanumWheelDrive(float strafeDirection, float strafeThrottle, float turnDirection, float turnThrottle) {
        strafeDirection = Range.clip(strafeDirection, -1, 1);
        strafeThrottle = Range.clip(strafeThrottle, -1, 1);
        turnThrottle = Range.clip(turnThrottle, -1, 1);
        turnDirection = Range.clip(turnDirection, -1, 1);

        float frontLeftPwr = (Range.clip(strafeThrottle + strafeDirection, -1, 1) - Range.clip(turnThrottle + turnDirection, -1, 1));
        float frontRightPwr = (Range.clip(strafeDirection - strafeThrottle, -1, 1) + Range.clip(turnThrottle - turnDirection, -1, 1));
        float rearLeftPwr = Range.clip(strafeDirection - strafeThrottle, -1, 1) + Range.clip(turnThrottle + turnDirection, -1, 1);
        float rearRightPwr = Range.clip(strafeThrottle + strafeDirection, -1, 1) - Range.clip(turnThrottle - turnDirection, -1, 1);

        this.powerTrain[0].motor.setPower(Range.clip(frontLeftPwr, -1, 1));
        this.powerTrain[1].motor.setPower(Range.clip(rearRightPwr, -1, 1));
        this.powerTrain[2].motor.setPower(Range.clip(frontRightPwr, -1, 1));
        this.powerTrain[3].motor.setPower(Range.clip(rearLeftPwr, -1, 1));

    }
}
