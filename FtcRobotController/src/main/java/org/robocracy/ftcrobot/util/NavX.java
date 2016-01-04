package org.robocracy.ftcrobot.util;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.robocracy.ftcrobot.FTCRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * @author Team Robocracy
 *
 * Enables NavX Micro functionality.
 */
public class NavX {
    private String startDate;
    private ElapsedTime runtime;
    FTCRobot robot;
    LinearOpMode curOpMode;
    AHRS navx;
    float[] navx_data;

    public NavX(FTCRobot robot, LinearOpMode curOpMode, AHRS navx){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navx = navx;
        this.navx_data = new float[5];
        this.runtime = new ElapsedTime();
    }

    /**
     * Gets processed data of NavX Micro device.
     * @return array of NavX Micro processed data
     */
    public float[] getNavxData(){
        navx_data[0] = navx.getYaw();
        navx_data[1] = navx.getPitch();
        navx_data[2] = navx.getRoll();
        navx_data[3] = navx.getCompassHeading();
        navx_data[4] = navx.getAltitude();

        return navx_data;
    }
}
