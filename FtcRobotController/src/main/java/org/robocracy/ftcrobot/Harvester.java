package org.robocracy.ftcrobot;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * @author Team Robocracy
 *
 * Operates harvester.
 */
public class Harvester {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor harvesterMotor;
    boolean harvesterAvailable=false;
    double maxSpeed = 152;

    public Harvester(FTCRobot robot, LinearOpMode curOpMode, DcMotor harvesterMotor){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.harvesterMotor = harvesterMotor;
        if (harvesterMotor != null) {
            harvesterAvailable = true;
        }
    }


    /**
     * Applies power to harvester motor based on {@code enum direction} value set in {@link DriverStation#getNextCommand()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        if (!harvesterAvailable) {
            return;
        }
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
