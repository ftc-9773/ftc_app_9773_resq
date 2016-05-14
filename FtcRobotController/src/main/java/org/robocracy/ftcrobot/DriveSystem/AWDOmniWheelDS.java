package org.robocracy.ftcrobot.DriveSystem;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.LinearLift;
import org.robocracy.ftcrobot.util.FileRW;
import org.robocracy.ftcrobot.util.PIDController;

/**
 * @author Team Robocracy
 *
 * This class implements the drive system with the following characteristics:
 * 1) The drive system has of 4 wheels.
 * 2) All the 4 wheels are Omni wheels
 * 3) Each of the 4 mecanum wheels is connected to a separate DC motor
 * 4) Front and Rear wheels are directly connected to their motors
 * 5) Left and Right wheels are connected to their motors via 1:1 gear ratio.
 */
public class AWDOmniWheelDS {
    // Index convention for motors, wheels, and other variables:
    // 0: Left
    // 1: Right
    // 2: Front
    // 3: Rear
    public PowerTrain[] powerTrain;
    private ElapsedTime runtime;
    private final double MIN_MOTOR_OUTPUT_VALUE;
    private final double MAX_MOTOR_OUTPUT_VALUE;
    LinearOpMode curOpmode;
    double robotMaxSpeed;
    // robotLength = Distance in inches from the center of front left to the center of rear left wheel
    // robotWidth = Distance in inches from the center of front left to the center of front right wheel
    double robotLength, robotWidth;
    FTCRobot robot;
    LinearLift linearLift;
    private static final int LEFT = 0;
    private static final int RIGHT = 1;
    private static final int FRONT = 2;
    private static final int REAR = 3;

    public AWDOmniWheelDS(LinearOpMode myOpmode, FTCRobot robot) {
        double wheelDiameter, forwardFrictionCoeff, sidewaysFrictionCoeff;
        double[] gearRatio = new double[4];
        double[] efficiency = new double[4];

        this.curOpmode = myOpmode;

        this.robot = robot;

        DcMotor[] motors = new DcMotor[4];
        motors[LEFT] = myOpmode.hardwareMap.dcMotor.get("leftMotor");
        motors[RIGHT] = myOpmode.hardwareMap.dcMotor.get("rightMotor");
        motors[FRONT] = myOpmode.hardwareMap.dcMotor.get("frontMotor");
        motors[REAR] = myOpmode.hardwareMap.dcMotor.get("rearMotor");
        motors[LEFT].setDirection(DcMotor.Direction.REVERSE); //Front Right
        motors[REAR].setDirection(DcMotor.Direction.REVERSE); // Rear Right
/*
        for (int i = 0; i < 4; i++) {
            motors[i].setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
*/

        wheelDiameter = 4.0;
        forwardFrictionCoeff = 0.5;
        sidewaysFrictionCoeff = 0.0;
        Wheel[] wheels = new Wheel[4];
        for (int i = 0; i < 4; i++) {
            wheels[i] = new OmniWheel(wheelDiameter, forwardFrictionCoeff, sidewaysFrictionCoeff);
        }

        double motorEncoderCPR = 1120; // for AndyMark Neverest 40
        double motorSpeedMax = 160; // rpm for AndyMark Neverest 40
        double motorStallTorque = 350; // oz-in for AndyMark Neverest 40
        double motorOutputPower = 14; // Watts for AndyMark Neverest 40

        for (int i = 0; i < 4; i++) {
            gearRatio[i] = 1.0;
        }
        //        FLgearRatio = FRgearRatio = RLgearRatio = RRgearRatio = 1.0;
        // We assume that the motor power efficiency when connected through sprocket-chain is
        // 95% of the power delivered when directly connected.
        efficiency[0] = efficiency[1] = 1.0;
        efficiency[2] = efficiency[3] = 1.0;


        this.powerTrain = new PowerTrain[4];

        for(int i=0;i<4;i++){
            this.powerTrain[i] = new PowerTrain(wheels[i], gearRatio[i], motors[i], motorEncoderCPR,
                    motorSpeedMax, motorStallTorque, motorOutputPower, efficiency[i]);
        }


        // Determine the maxSpeed for this robot in inches/sec
        this.robotMaxSpeed = 100.0;
        for (int i = 0; i < 4; i++) {
            this.robotMaxSpeed = Math.min(this.robotMaxSpeed, this.powerTrain[i].wheelSpeedMax);
        }

        this.robotLength = 10.5; // in  inches
        this.robotWidth = 15; // in  inches
        this.linearLift = new LinearLift(robot, curOpmode);
        this.MAX_MOTOR_OUTPUT_VALUE = 1;
        this.MIN_MOTOR_OUTPUT_VALUE = -1;
        this.runtime = new ElapsedTime();
    }

    /**
     * Calculates speed of each wheel based on relative angle required to move in.
     * @param angle angle to move in relative to direction robot is facing
     * @param speed overall speed robot should move in
     * @param Omega angle to turn relative to 2 dimensional center of robot
     * @param speedOfWheel wheel speed array
     */
    private void angleToSpeedOfWheel(int angle, double speed, double Omega, double[] speedOfWheel) {
        double Vx, Vy;

        // Find the quadrant corresponding to the angle
        if (angle < 0) {
            angle = 360 + angle;
        }

        if (speed > 0) {
            if (angle == 0) {
                speedOfWheel[LEFT] = speedOfWheel[RIGHT] = 0.0;
                speedOfWheel[FRONT] = speedOfWheel[REAR] = speed;
            } else if (angle == 90) {
                speedOfWheel[LEFT] = speedOfWheel[RIGHT] = speed;
                speedOfWheel[FRONT] = speedOfWheel[REAR] = 0.0;
            } else if(angle == 180) {
                speedOfWheel[LEFT] = speedOfWheel[RIGHT] = 0.0;
                speedOfWheel[FRONT] = speedOfWheel[REAR] = -speed;
            } else if (angle == 270) {
                speedOfWheel[LEFT] = speedOfWheel[RIGHT] = -speed;
                speedOfWheel[FRONT] = speedOfWheel[REAR] = 0.0;
            }
        } else {
            if (Omega != 0) {
                speedOfWheel[RIGHT] = speedOfWheel[FRONT] = this.robotMaxSpeed * Omega;
                speedOfWheel[LEFT] = speedOfWheel[REAR] = -1 * this.robotMaxSpeed * Omega;
            }
        }
    }

    // angle is specified in degres:  -360 < angle < 360


    /**
     * Applies {@link DriverCommand#drvsyscmd} values initialized in {@link DriverStation#getNextDrivesysCmd()} to {@link AWDOmniWheelDS#driveMecanum(int, double, double)}
     * @param driverCommand {@link DriverCommand} object with values
     * @throws InterruptedException
     */
    public void applyCmd(DriverCommand driverCommand) throws InterruptedException {
        this.driveMecanum((int) driverCommand.drvsyscmd.angle, driverCommand.drvsyscmd.speedMultiplier, driverCommand.drvsyscmd.Omega);
    }

    /**
     * Calculates and applies power to each motor to move robot in specified angle, at specified speed
     * @param angle angle robot should move in relative to direction robot is facing
     * @param speedMultiplier overall speed robot should move in
     * @param Omega degrees robot should turn relative to 2 dimensional center of robot
     * @throws InterruptedException
     * @see  FileRW
     */
    private void driveMecanum(int angle, double speedMultiplier, double Omega) throws  InterruptedException{
        double[] speedOfWheel = new double[4];
        double[] motorPower = new double[4];

        //Error checks
        if ((angle <= -360) || (angle >= 360)) {
            DbgLog.error(String.format("Invalid angle %d\n", angle));
            return;
        }

        if (speedMultiplier < 0 || speedMultiplier > 1) {
            DbgLog.error(String.format("Invalid speedMultiplier %f\n", speedMultiplier));
            return;
        }

        double speed = speedMultiplier * this.robotMaxSpeed;
        this.angleToSpeedOfWheel(angle, speed, Omega, speedOfWheel);

        // Determine the Power for each Motor/PowerTrain
        for (int i=0; i<4; i++) {
            motorPower[i] = speedOfWheel[i] * this.powerTrain[i].motorPowerMultiplier;
            motorPower[i] = Range.clip(motorPower[i], -1, 1);
            this.powerTrain[i].motor.setPower(motorPower[i]);
        }
    }


    public void stopDriveSystem() {
        for (int i = 0; i < 4; i++) {
            this.powerTrain[i].motor.setPower(0);
        }
    }
}
