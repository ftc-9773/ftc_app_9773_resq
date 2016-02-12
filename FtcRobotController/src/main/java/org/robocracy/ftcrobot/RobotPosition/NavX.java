package org.robocracy.ftcrobot.RobotPosition;

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
    public float[] initial_navx_data;
    boolean navxIsAvailable = false;

    public NavX(FTCRobot robot, LinearOpMode curOpMode, AHRS navx) throws InterruptedException {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navx = navx;
        if (navx == null) {
            return;
        }
        navxIsAvailable = true;
        this.navx_data = new float[5];
        this.runtime = new ElapsedTime();

        // Wait till navX is done with Calibrating
        if (navx.isConnected()) {
            while (navx.isCalibrating()) {
                curOpMode.waitForNextHardwareCycle();
            }
        }
        initial_navx_data = new float[5];
        initial_navx_data[0] = navx.getYaw();
        initial_navx_data[1] = navx.getPitch();
        initial_navx_data[2] = navx.getRoll();
        initial_navx_data[3] = navx.getCompassHeading();
        initial_navx_data[4] = navx.getAltitude();

    }

    /**
     * Gets processed data of NavX Micro device.
     * @return array of NavX Micro processed data
     */
    public float[] getNavxData(){
        if (! navxIsAvailable) {
            return navx_data;
        }
        navx_data[0] = navx.getYaw();
        navx_data[1] = navx.getPitch();
        navx_data[2] = navx.getRoll();
        navx_data[3] = navx.getCompassHeading();
        navx_data[4] = navx.getAltitude();

        return navx_data;
    }
}
