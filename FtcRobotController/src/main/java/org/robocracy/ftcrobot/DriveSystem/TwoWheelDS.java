package org.robocracy.ftcrobot.DriveSystem;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.LinearLift;
import org.robocracy.ftcrobot.util.PIDController;

/**
 * Created by pranavb on 4/10/16.
 */
public class TwoWheelDS {
    // Index convention for motors, wheels, and other variables:
    // 0: Left
    // 1: Right
    public PowerTrain[] powerTrain;
    PIDController[] motorPIDController;
    private ElapsedTime runtime;
    LinearOpMode curOpmode;
    double robotMaxSpeed;
    // robotLength = Distance in inches from the center of front left to the center of rear left wheel
    // robotWidth = Distance in inches from the center of front left to the center of front right wheel
    double robotLength, robotWidth;
    FTCRobot robot;
    LinearLift linearLift;

    public TwoWheelDS(LinearOpMode curOpmode, FTCRobot robot) {
        double wheelDiameter, forwardFrictionCoeff;
        double[] gearRatio = new double[4];
        double[] efficiency = new double[4];

        this.curOpmode = curOpmode;

        this.robot = robot;

        DcMotor[] motors = new DcMotor[2];
        motors[0] = curOpmode.hardwareMap.dcMotor.get("motorL");
        motors[1] = curOpmode.hardwareMap.dcMotor.get("motorR");
        motors[1].setDirection(DcMotor.Direction.REVERSE); //Right
        for (int i = 0; i < 2; i++) {
            motors[i].setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        wheelDiameter = 4.0;
        forwardFrictionCoeff = 0.5;
        Wheel[] wheels = new Wheel[2];
        for (int i = 0; i < 2; i++) {
            wheels[i] = new TetrixWheel(wheelDiameter, forwardFrictionCoeff);
        }

        double motorEncoderCPR = 1120; // for AndyMark Neverest 40
        double motorSpeedMax = 160; // rpm for AndyMark Neverest 40
        double motorStallTorque = 350; // oz-in for AndyMark Neverest 40
        double motorOutputPower = 14; // Watts for AndyMark Neverest 40

        for (int i = 0; i < 2; i++) {
            gearRatio[i] = 1.0;
        }
        //        LgearRatio = RgearRatio = 1.0;
        // We assume that the motor power efficiency when connected through sprocket-chain is
        // 95% of the power delivered when directly connected.
        efficiency[0] = efficiency[1] = 1.0;


        this.powerTrain = new PowerTrain[2];

        for(int i=0;i<2;i++){
            this.powerTrain[i] = new PowerTrain(wheels[i], gearRatio[i], motors[i], motorEncoderCPR,
                    motorSpeedMax, motorStallTorque, motorOutputPower, efficiency[i]);
        }


        // Determine the maxSpeed for this robot in inches/sec
        this.robotMaxSpeed = 100.0;
        for (int i = 0; i < 2; i++) {
            this.robotMaxSpeed = Math.min(this.robotMaxSpeed, this.powerTrain[i].wheelSpeedMax);
        }

        this.motorPIDController = new PIDController[2];
        for (int i = 0; i < 2; i++) {
            // ToDo:  Determine the corect values for Kp, Ki, and Kd and pass them in the constructor below.
            this.motorPIDController[i] = new PIDController(0, 0, 0, 0, 0, 0);
        }
        this.robotLength = 10.5; // in  inches
        this.robotWidth = 15; // in  inches
    }

    public void applyCmd(DriverCommand driverCommand) throws InterruptedException {
        this.drive(driverCommand.drvsyscmd.speedMultiplier, driverCommand.drvsyscmd.Omega);

    }

    public void drive(double speedMultiplier, double Omega) throws InterruptedException{
        double[] speedOfWheel = new double[2];
        double[] motorPower = new double[2];
        if (speedMultiplier < -1 || speedMultiplier > 1) {
            DbgLog.error(String.format("Invalid speedMultiplier %f\n", speedMultiplier));
            return;
        }
        double speed = speedMultiplier * this.robotMaxSpeed;
        for (int i=0; i<2; i++) {
            motorPower[i] = speed * this.powerTrain[i].motorPowerMultiplier;
            motorPower[i] = Range.clip(motorPower[i], -1, 1);
            this.powerTrain[i].motor.setPower(motorPower[i]);
        }
    }

    public void stopDriveSystem() {
        for (int i = 0; i < 2; i++) {
            this.powerTrain[i].motor.setPower(0);
        }
    }
}
