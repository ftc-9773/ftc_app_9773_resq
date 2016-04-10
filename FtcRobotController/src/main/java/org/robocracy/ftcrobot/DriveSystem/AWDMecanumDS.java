package org.robocracy.ftcrobot.DriveSystem;

import com.kauailabs.navx.ftc.navXPIDController;
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
 * 2) All the 4 wheels are mecanum wheels
 * 3) Each of the 4 mecanum wheels is connected to a separate DC motor
 * 4) Rear wheels are directly connected to their motors
 * 5) Front wheels are connected to their motors via chain and sprocket.
 */
public class AWDMecanumDS {
    // Index convention for motors, wheels, and other variables
    // 0: Front Left
    // 1: Front Right
    // 2: Rear Left
    // 3: Rear Right
    public PowerTrain[] powerTrain;
    PIDController[] motorPIDController;
    navXPIDController yawPIDController;
    private ElapsedTime runtime;
    navXPIDController.PIDResult yawPIDResult;
    private final double MIN_MOTOR_OUTPUT_VALUE;
    private final double MAX_MOTOR_OUTPUT_VALUE;
    LinearOpMode curOpmode;
    double robotMaxSpeed;
    // robotLength = Distance in inches from the center of front left to the center of rear left wheel
    // robotWidth = Distance in inches from the center of front left to the center of front right wheel
    double robotLength, robotWidth;
    FTCRobot robot;
    LinearLift linearLift;

    public AWDMecanumDS(LinearOpMode myOpmode, FTCRobot robot) {
        double wheelDiameter, forwardFrictionCoeff, sidewaysFrictionCoeff;
        double[] gearRatio = new double[4];
        double[] efficiency = new double[4];

        this.curOpmode = myOpmode;

        this.robot = robot;

        DcMotor[] motors = new DcMotor[4];
        motors[0] = myOpmode.hardwareMap.dcMotor.get("fMotorL");
        motors[1] = myOpmode.hardwareMap.dcMotor.get("fMotorR");
        motors[2] = myOpmode.hardwareMap.dcMotor.get("rMotorL");
        motors[3] = myOpmode.hardwareMap.dcMotor.get("rMotorR");
        motors[1].setDirection(DcMotor.Direction.REVERSE); //Front Right
        motors[3].setDirection(DcMotor.Direction.REVERSE); // Rear Right
        for (int i = 0; i < 4; i++) {
            motors[i].setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        wheelDiameter = 4.0;
        forwardFrictionCoeff = 0.5;
        sidewaysFrictionCoeff = 0.6;
        Wheel[] wheels = new Wheel[4];
        for (int i = 0; i < 4; i++) {
            wheels[i] = new MecanumWheel(wheelDiameter, forwardFrictionCoeff, sidewaysFrictionCoeff);
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

        this.motorPIDController = new PIDController[4];
        for (int i = 0; i < 4; i++) {
            // ToDo:  Determine the corect values for Kp, Ki, and Kd and pass them in the constructor below.
            this.motorPIDController[i] = new PIDController(0, 0, 0, 0, 0, 0);
        }
        this.robotLength = 10.5; // in  inches
        this.robotWidth = 15; // in  inches
        this.linearLift = new LinearLift(robot, curOpmode);
        this.yawPIDController = new navXPIDController(robot.navxDevice,
                navXPIDController.navXTimestampedDataSource.YAW);
        this.MAX_MOTOR_OUTPUT_VALUE = 1;
        this.MIN_MOTOR_OUTPUT_VALUE = -1;
        this.runtime = new ElapsedTime();
        this.yawPIDResult = new navXPIDController.PIDResult();
    }

    public void close() {
        this.yawPIDController.close();
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
        int quadrant = angle / 90;
        int angleInQuadrant = angle % 90;
        // If the angle is in 1st or 3rd quadrants
        if (quadrant == 0 || quadrant == 2) {
            Vx = Math.cos(Math.toRadians((double) angleInQuadrant)) * speed;
            Vy = Math.sin(Math.toRadians((double) angleInQuadrant)) * speed;
        } else {
            Vy = Math.cos(Math.toRadians((double) angleInQuadrant)) * speed;
            Vx = Math.sin(Math.toRadians((double) angleInQuadrant)) * speed;
        }

        if (quadrant == 1) {
            Vx = -Vx;
        } else if (quadrant == 2) {
            Vx = -Vx;
            Vy = -Vy;
        } else if (quadrant == 3) {
            Vy = -Vy;
        }
        //Calculate the speed of each wheel
        // Reference: Omni Drive system presentation by Andy Baker and Ian Mackenzie
        speedOfWheel[0] = Vy + Vx - (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[1] = Vy - Vx + (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[2] = Vy - Vx - (Omega * (this.robotWidth + this.robotLength) / 2);
        speedOfWheel[3] = Vy + Vx + (Omega * (this.robotWidth + this.robotLength) / 2);

    }

    // angle is specified in degres:  -360 < angle < 360

    /**
     * Moves robot at specified angle, spinning the robot at the specified degrees, for a specified distance at a specified speed
     * @param angle angle robot should move in relative to direction robot is facing
     * @param distance distance in inches of the vector robot should move in
     * @param speed overall speed of robot
     * @param robotSpinDegrees degrees robot should spin while moving
     * @throws InterruptedException
     * @see AWDMecanumDS#driveMecanum(int, double, double)
     */
    public void autoMecanum(int angle, int distance, double speed, int robotSpinDegrees) throws InterruptedException {
        double[] distanceTravelledByWheel = new double[4];
        double[] motorPower = new double[4];
        int[] motorPosition = new int[4];
        int[] prevPosition = new int[4];
        double Omega;
        double[] speedOfWheel = new double[4];
        double correction = 0.0;

        //Various error checks
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

        // Convert robotSpinDegrees into angular velocity
        Omega = Math.toRadians(robotSpinDegrees) / (speed / distance);

        this.angleToSpeedOfWheel(angle, speed, Omega, speedOfWheel);

        for (int i = 0; i < 4; i++) {
            this.motorPIDController[i].setSetPoint(speedOfWheel[i]);
        }
        for (int i = 0; i < 4; i++) {
            prevPosition[i] = this.powerTrain[i].motor.getCurrentPosition();
        }
        // It is assumed that all the motors are already set to USE_ENCODER mode
        // during initialization
        long startTime = System.nanoTime();
        long endTime = 0;
        double elapsedTime = 0;
        double distanceTravelled = 0.0;

        // Determine the Power for each Motor/PowerTrain
        for (int i = 0; i < 4; i++) {
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
                correction = this.motorPIDController[i].getCorrection(distanceTravelledByWheel[i] / elapsedTime);
                this.powerTrain[i].motor.setPower(Range.clip((motorPower[i] - correction), -1, 1));
            }
            System.arraycopy(motorPosition, 0, prevPosition, 0, 4);
            distanceTravelled += (Math.abs(distanceTravelledByWheel[0]) +
                    Math.abs(distanceTravelledByWheel[1]) +
                    Math.abs(distanceTravelledByWheel[2]) +
                    Math.abs(distanceTravelledByWheel[3])) / 4;
        }

        for (int i = 0; i < 4; i++) {
            this.powerTrain[i].motor.setPower(0);
        }

        // Wait for one hardware cycle for the setPower(0) to take effect.
        this.curOpmode.waitForNextHardwareCycle();
    }

    /**
     * Applies {@link DriverCommand#drvsyscmd} values initialized in {@link DriverStation#getNextDrivesysCmd()} to {@link AWDMecanumDS#driveMecanum(int, double, double)}
     * @param driverCommand {@link DriverCommand} object with values
     * @throws InterruptedException
     */
    public void applyCmd(DriverCommand driverCommand) throws InterruptedException {
        if(driverCommand.drvsyscmd.angle == 0 || driverCommand.drvsyscmd.angle == 180){
            driverCommand.drvsyscmd.speedMultiplier *= 0.5;
        }
        this.driveMecanum((int) driverCommand.drvsyscmd.angle, driverCommand.drvsyscmd.speedMultiplier, driverCommand.drvsyscmd.Omega);

    }

    /**
     * Calculates and applies power to each motor to move robot in specified angle, at specified speed
     * @param angle angle robot should move in relative to direction robot is facing
     * @param speedMultiplier overall speed robot should move in
     * @param Omega degrees robot should turn relative to 2 dimensional center of robot
     * @throws InterruptedException
     * @see AWDMecanumDS#autoMecanum(int, int, double, int)
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

    /**
     * PID controller that uses the yaw value from the navX sensor to move straight
     * @param YAW_PID_P Proportional multiplier constant
     * @param YAW_PID_I Integral multiplier constant
     * @param YAW_PID_D Derivative multiplier constant
     * @throws InterruptedException
     */
    public void PIDmoveStraight(double YAW_PID_P, double YAW_PID_I, double YAW_PID_D) throws InterruptedException{
        final double TOLERANCE_DEGREES = 2.0;

        yawPIDController.setSetpoint(robot.navxDevice.getYaw());
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        if (yawPIDResult.isOnTarget()) {
            robot.driveSys.driveMecanum(90, -6, 0);
        } else {
            double output = yawPIDResult.getOutput();
            robot.driveSys.driveMecanum(90, -6, output);
        }
    }

    /**
     * PID controller that overrules {@link AWDMecanumDS#PIDmoveStraight(double, double, double)}. Uses the yaw value of the
     * navX sensor to move straight in all directions.
     * @param YAW_PID_P Proportional multiplier constant
     * @param YAW_PID_I Integral multiplier constant
     * @param YAW_PID_D Derivative multiplier constant
     * @param strafeAngle Angle at which to strafe
     * @throws InterruptedException
     */
    public void PIDmoveStraight(double YAW_PID_P, double YAW_PID_I, double YAW_PID_D, int strafeAngle, double distance) throws InterruptedException{
        final double TOLERANCE_DEGREES = 2.0;
        final int DEVICE_TIMEOUT_MS=100; // timeout in milli seconds
        double distTraveled = 0.0;
        double[] distanceTravelledByWheel = new double[4];
        int[] motorPosition = new int[4];
        int[] prevPosition = new int[4];


        yawPIDController.reset();
        yawPIDController.setSetpoint(robot.navxDevice.getYaw());
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        for (int i = 0; i < 4; i++) {
            prevPosition[i] = this.powerTrain[i].motor.getCurrentPosition();
        }

        while (curOpmode.opModeIsActive() && (Math.abs(distTraveled) < distance)) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {

                if (yawPIDResult.isOnTarget()) {
                    robot.driveSys.driveMecanum(strafeAngle, 6, 0);
                } else {
                    double output = yawPIDResult.getOutput();
                    robot.driveSys.driveMecanum(strafeAngle, 6, output);
                    DbgLog.msg(String.format("PIDoutput: %f", output));
                }

                for (int i = 0; i < 4; i++) {
                    motorPosition[i] = this.powerTrain[i].motor.getCurrentPosition();
                    distanceTravelledByWheel[i] = this.powerTrain[i].countsToDistance((motorPosition[i] - prevPosition[i]));
                }
                System.arraycopy(motorPosition, 0, prevPosition, 0, 4);
                distTraveled += (Math.abs(distanceTravelledByWheel[0]) +
                        Math.abs(distanceTravelledByWheel[1]) +
                        Math.abs(distanceTravelledByWheel[2]) +
                        Math.abs(distanceTravelledByWheel[3])) / 4;

                curOpmode.waitForNextHardwareCycle();
            }
        }
        yawPIDController.enable(false);
    }

    public void rotateToAngle(float targetYaw) throws InterruptedException {
        DriverCommand tmpDrvrCmd = new DriverCommand();
        float initialYaw = robot.navxDevice.getYaw();
        float angleToSpin = Math.abs(initialYaw - targetYaw);
        int directionIndicator;
        boolean reachedTargetYaw = false;

        DbgLog.msg(String.format("TargetYaw=%f, initialYaw=%f", targetYaw, initialYaw));

        directionIndicator = (targetYaw > robot.navxDevice.getYaw() ? -1 : 1);
        while (!reachedTargetYaw) {
            tmpDrvrCmd.drvsyscmd.angle = 0;
            tmpDrvrCmd.drvsyscmd.speedMultiplier = 0;
            tmpDrvrCmd.drvsyscmd.Omega = directionIndicator * 0.5;
            this.applyCmd(tmpDrvrCmd);
            if (Math.abs(robot.navxDevice.getYaw() - initialYaw) >= angleToSpin) {
                reachedTargetYaw = true;
            }
            this.curOpmode.waitForNextHardwareCycle();
        }
        // Now stop the robot
        tmpDrvrCmd.drvsyscmd.angle = 0;
        tmpDrvrCmd.drvsyscmd.speedMultiplier = 0;
        tmpDrvrCmd.drvsyscmd.Omega = 0;
        this.applyCmd(tmpDrvrCmd);
        this.curOpmode.waitForNextHardwareCycle();
    }

    public void stopDriveSystem() {
        for (int i = 0; i < 4; i++) {
            this.powerTrain[i].motor.setPower(0);
        }
    }
}
