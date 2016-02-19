package org.robocracy.ftcrobot;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.util.FileRW;

import java.io.IOException;

/**
 * Operates Linear Lift on robot.
 * @author Team Robocracy
 */
public class LinearLift {
    FTCRobot robot;
    LinearOpMode curOpMode;
    String filePath;
    String[] line, lineArray;
    FileRW fileRW;
    public DcMotor liftAngleMotor;
    public DcMotor liftArmLengthMotor;
    boolean liftAngleMotorAvailable = false;
    boolean liftArmLengthMotorAvailable = false;
    boolean armLengthSafe, angleSafe = true;
    double armLengthEncoder, angleEncoder, armLengthEncoderMax, armLengthEncoderMin, armLengthEncoderStarting,
            armLengthEncoderHigh, armLengthEncoderMedium, armLengthEncoderEndGame,
            angleEncoderMax, angleEncoderMin, angleEncoderStarting, angleEncoderHigh, angleEncoderMedium,
            angleEncoderEndGame;

    public LinearLift(FTCRobot robot, LinearOpMode curOpMode){
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
        this.armLengthEncoder = liftArmLengthMotor.getCurrentPosition();
        this.armLengthEncoder = liftAngleMotor.getCurrentPosition();
        this.filePath = "/sdcard/FIRST/liftEncoderValues.csv";
        this.fileRW = new FileRW(filePath, false);


        line[0] = fileRW.getNextLine();
        lineArray = line[0].split(",");
        armLengthEncoderStarting = Double.parseDouble(lineArray[0]);
        angleEncoderStarting = Double.parseDouble(lineArray[1]);

        line[1] = fileRW.getNextLine();
        lineArray = line[1].split(",");
        armLengthEncoderMin = Double.parseDouble(lineArray[0]);
        angleEncoderMin = Double.parseDouble(lineArray[1]);

        line[2] = fileRW.getNextLine();
        lineArray = line[2].split(",");
        armLengthEncoderMax = Double.parseDouble(lineArray[0]);
        angleEncoderMax = Double.parseDouble(lineArray[1]);

        line[3] = fileRW.getNextLine();
        lineArray = line[3].split(",");
        armLengthEncoderMedium = Double.parseDouble(lineArray[0]);
        angleEncoderMedium = Double.parseDouble(lineArray[1]);

        line[4] = fileRW.getNextLine();
        lineArray = line[4].split(",");
        armLengthEncoderHigh = Double.parseDouble(lineArray[0]);
        angleEncoderHigh = Double.parseDouble(lineArray[1]);

        line[5] = fileRW.getNextLine();
        lineArray = line[5].split(",");
        armLengthEncoderEndGame = Double.parseDouble(lineArray[0]);
        angleEncoderEndGame = Double.parseDouble(lineArray[1]);

        try {
            if (fileRW != null){
                fileRW.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void runLiftMotorsUsingEncoders(double liftArmLengthEncoderValue, double liftAngleEncoderValue){
        liftArmLengthMotor.setTargetPosition((int) liftArmLengthEncoderValue);
        liftAngleMotor.setTargetPosition((int) liftAngleEncoderValue);
        liftArmLengthMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        liftAngleMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        while(!(Math.abs(liftAngleMotor.getCurrentPosition())>=liftAngleEncoderValue) && !((Math.abs(liftArmLengthMotor.getCurrentPosition())>=liftArmLengthEncoderValue))){
            liftAngleMotor.setPower(1);
            liftArmLengthMotor.setPower(1);
        }
    }

    /**
     * Applies power to lift motors based on value in {@code double direction, angle} set in {@link DriverStation#getNextDrivesysCmd()}.
     * @param driverCommand {@link DriverCommand} object with values.
     */
    public void applyCmd(DriverCommand driverCommand) throws InterruptedException{
        if(armLengthEncoder<=armLengthEncoderMin || armLengthEncoder>=armLengthEncoderMax){
            armLengthSafe = false;
        }
        if(angleEncoder<=angleEncoderMin || angleEncoder>=angleEncoderMax){
            angleSafe = false;
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

        switch (driverCommand.linliftcmd.linLiftGoToPosition){
            case STARTING:
                runLiftMotorsUsingEncoders(liftArmLengthMotor.getCurrentPosition(), angleEncoderMax);
                runLiftMotorsUsingEncoders(armLengthEncoderStarting, angleEncoderStarting);
                curOpMode.waitOneFullHardwareCycle();

                break;
            case MEDIUM_GOAL:
                runLiftMotorsUsingEncoders(liftArmLengthMotor.getCurrentPosition(), angleEncoderMax);
                runLiftMotorsUsingEncoders(armLengthEncoderMedium, angleEncoderMedium);
                break;
            case HIGH_GOAL:
                runLiftMotorsUsingEncoders(liftArmLengthMotor.getCurrentPosition(), angleEncoderMax);
                runLiftMotorsUsingEncoders(armLengthEncoderHigh, angleEncoderHigh);
                break;
            case END_GAME:
                runLiftMotorsUsingEncoders(liftArmLengthMotor.getCurrentPosition(), angleEncoderMax);
                runLiftMotorsUsingEncoders(armLengthEncoderEndGame, angleEncoderEndGame);
                break;
            case MAX:
                runLiftMotorsUsingEncoders(liftArmLengthMotor.getCurrentPosition(), angleEncoderMax);
                runLiftMotorsUsingEncoders(armLengthEncoderMax, angleEncoderMax);
                break;
            case MIN:
                runLiftMotorsUsingEncoders(liftArmLengthMotor.getCurrentPosition(), angleEncoderMax);
                runLiftMotorsUsingEncoders(armLengthEncoderMin, angleEncoderMin);
                break;
            case NONE:
                break;

        }

        armLengthEncoder = liftArmLengthMotor.getCurrentPosition();
        angleEncoder = liftAngleMotor.getCurrentPosition();
    }
}
