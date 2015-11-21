package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.robocracy.ftcrobot.FTCRobot;

/**
 * Created by Robocracy on 11/19/2015.
 */
public class TeleOpNew extends LinearOpMode {
    FTCRobot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.myRobot = new FTCRobot(this);


        waitOneFullHardwareCycle();

        waitForStart();

        myRobot.runRobotAutonomous();
    }
}
