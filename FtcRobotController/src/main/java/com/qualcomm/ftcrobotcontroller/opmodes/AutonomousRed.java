package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.FTCRobot;

/*import com.qualcomm.robotcore.eventloop.opmode.OpMode;*/

/**
 * Created by pranavb on 11/11/15.
 */
public class AutonomousRed extends LinearOpMode {
    FTCRobot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.myRobot = new FTCRobot(this, false);


        waitOneFullHardwareCycle();

        waitForStart();

        myRobot.runRobotAutonomous();

    }
}
