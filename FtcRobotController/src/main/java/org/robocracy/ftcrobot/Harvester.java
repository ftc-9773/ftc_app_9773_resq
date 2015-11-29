package org.robocracy.ftcrobot;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.FTCRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by pranavb on 11/29/15.
 */
public class Harvester {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor harvesterMotor;
    double maxSpeed = 152;

    public Harvester(FTCRobot robot, LinearOpMode curOpMode, DcMotor harvesterMotor){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.harvesterMotor = harvesterMotor;
    }


    public void applyDSCmd(DriverCommand drvrcmd){
        switch (drvrcmd.harvestercmd.direction){
            case PULL:
                harvesterMotor.setPower(1);
                break;
            case PUSH:
                harvesterMotor.setPower(-0.3);
                break;
            case NONE:
                harvesterMotor.setPower(0);
                break;
            default:
                harvesterMotor.setPower(0);
                break;
        }
    }
}
