package org.robocracy.ftcrobot.RobotPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.robocracy.ftcrobot.FTCRobot;

import java.io.OptionalDataException;

/**
 * Created by Robocracy on 2/10/2016.
 */
public class RobotPosition {
    float[] navx_data;
    double ods_lightDetected;
    int cs_red, cs_green, cs_blue;

    FTCRobot robot;
    OpMode curOpmode;

    public RobotPosition(FTCRobot robot, OpMode curOpmode, NavX navX,
                         ColorSensor cs, OpticalDistanceSensor ods) {
        this.robot = robot;
        this.curOpmode = curOpmode;
        this.navx_data = new float[5];
        this.navx_data = navX.getNavxData();
    }
}
