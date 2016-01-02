package org.robocracy.ftcrobot.util;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.robocracy.ftcrobot.FTCRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * Created by pranavb on 12/24/15.
 */
public class NavX {
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    FTCRobot robot;
    LinearOpMode curOpMode;
    AHRS navx;

    public NavX(FTCRobot robot, LinearOpMode curOpMode, AHRS navx){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navx = navx;
    }

    public double[] getNavxData(){
        double[] navx_data = new double[4];

        navx_data[0] = navx.getYaw();
        navx_data[1] = navx.getPitch();
        navx_data[2] = navx.getRoll();
        navx_data[3] = navx.getCompassHeading();
        navx_data[4] = navx.getAltitude();

        return navx_data;
    }
}
