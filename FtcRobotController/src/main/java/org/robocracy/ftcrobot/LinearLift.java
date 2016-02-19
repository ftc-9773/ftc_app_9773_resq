package org.robocracy.ftcrobot;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
/**
 * Operates Linear Lift on robot.
 * @author Team Robocracy
 */
public class LinearLift {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor liftAngleMotor;
    DcMotor liftArmLengthMotor;
    boolean liftAngleMotorAvailable = false;
    boolean liftArmLengthMotorAvailable = false;
    boolean armLengthSafe, angleSafe = true;
    double armLengthEncoder, angleEncoder, armLengthEncoderMax, armLengthEncoderMin, angleEncoderMax, angleEncoderMin;

    public LinearLift(FTCRobot robot, LinearOpMode curOpMode, double armLengthEncoderMax, double armLengthEncoderMin, double angleEncoderMax, double angleEncoderMin){
        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            this.liftAngleMotor = curOpMode.hardwareMap.dcMotor.get("liftAngleMotor");
            liftAngleMotorAvailable = true;
        }
        catch(Exception e){
            DbgLog.error(String.format("%s . Device skipped", e.getMessage()));
        }
        try {
            this.liftArmLengthMotor = curOpMode.hardwareMap.dcMotor.get("liftArmLengthMotor");
            liftArmLengthMotorAvailable = true;
        }
        catch(Exception e){
            DbgLog.error(String.format("%s . Device skipped", e.getMessage()));
        }
        this.armLengthEncoderMax = armLengthEncoderMax;
        this.armLengthEncoderMin = armLengthEncoderMin;
        this.angleEncoderMax = angleEncoderMax;
        this.angleEncoderMin = angleEncoderMin;
        this.armLengthEncoder = liftArmLengthMotor.getCurrentPosition();
        this.armLengthEncoder = liftAngleMotor.getCurrentPosition();
    }

    /**
     * Applies power to lift motors based on value in {@code double direction, angle} set in {@link DriverStation#getNextDrivesysCmd()}.
     * @param driverCommand {@link DriverCommand} object with values.
     */
    public void applyCmd(DriverCommand driverCommand){
        if(armLengthEncoder<=armLengthEncoderMin || armLengthEncoder>=armLengthEncoderMax || angleEncoder<=angleEncoderMin
                || angleEncoder>=angleEncoderMax){
            armLengthSafe = false;
        }

        if (liftArmLengthMotorAvailable && armLengthSafe) {
            liftArmLengthMotor.setPower(driverCommand.linliftcmd.armLength);
        }
        else if(liftArmLengthMotorAvailable && !armLengthSafe && armLengthEncoder<=armLengthEncoderMin){
            if(driverCommand.linliftcmd.armLength < 0){
                liftArmLengthMotor.setPower(0);
            }
            else if(driverCommand.linliftcmd.armLength > 0){
                liftArmLengthMotor.setPower(driverCommand.linliftcmd.armLength);
            }
        }
        else if(liftArmLengthMotorAvailable && !armLengthSafe && armLengthEncoder>=armLengthEncoderMax){
            if(driverCommand.linliftcmd.armLength > 0){
                liftArmLengthMotor.setPower(0);
            }
            else if(driverCommand.linliftcmd.armLength < 0){
                liftArmLengthMotor.setPower(driverCommand.linliftcmd.armLength);
            }
        }

        if (liftAngleMotorAvailable && angleSafe) {
            liftAngleMotor.setPower(driverCommand.linliftcmd.angle);
        }
        else if(liftAngleMotorAvailable && !angleSafe && angleEncoder<=angleEncoderMin){
            if(driverCommand.linliftcmd.angle < 0){
                liftAngleMotor.setPower(0);
            }
            else if(driverCommand.linliftcmd.angle > 0){
                liftAngleMotor.setPower(driverCommand.linliftcmd.angle);
            }
        }
        else if(liftAngleMotorAvailable && !angleSafe && angleEncoder>=angleEncoderMax){
            if(driverCommand.linliftcmd.angle > 0){
                liftAngleMotor.setPower(0);
            }
            else if(driverCommand.linliftcmd.angle < 0){
                liftAngleMotor.setPower(driverCommand.linliftcmd.angle);
            }
        }

        armLengthEncoder = liftArmLengthMotor.getCurrentPosition();
        angleEncoder = liftAngleMotor.getCurrentPosition();
    }
}
