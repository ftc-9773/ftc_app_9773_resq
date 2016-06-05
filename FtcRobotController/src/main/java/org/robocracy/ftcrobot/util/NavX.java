package org.robocracy.ftcrobot.util;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.IDataArrivalSubscriber;
import com.qualcomm.ftccommon.DbgLog;
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

    static public class CollisionDetector implements IDataArrivalSubscriber {
        /* ToDo: Tune this threshold to adjust the sensitivy of the */
        /* Collision detection.                               */
        private final double COLLISION_THRESHOLD_DELTA_G = 0.5;

        double last_world_linear_accel_x;
        double last_world_linear_accel_y;
        private ElapsedTime runtime = new ElapsedTime();
        private AHRS navx_device;//navigation sensor
        private boolean collision_state;
        private long last_system_timestamp = 0;
        private long last_sensor_timestamp = 0;

        private long sensor_timestamp_delta = 0;
        private long system_timestamp_delta = 0;

        public CollisionDetector(AHRS navx_device) {
            this.navx_device = navx_device;
            last_world_linear_accel_x = 0.0;
            last_world_linear_accel_y = 0.0;
            setCollisionState(false);
            navx_device.registerCallback(this);
        }

        private void setCollisionState( boolean newValue ) {
            this.collision_state = newValue;
        }

        @Override
        public void timestampedDataReceived(long cur_system_timestamp, long cur_sensor_timestamp, Object kind) {
            boolean collisionDetected = false;

            sensor_timestamp_delta = cur_sensor_timestamp - last_sensor_timestamp;
            system_timestamp_delta = cur_system_timestamp - last_system_timestamp;
            double curr_world_linear_accel_x = navx_device.getWorldLinearAccelX();
            double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
            last_world_linear_accel_x = curr_world_linear_accel_x;
            double curr_world_linear_accel_y = navx_device.getWorldLinearAccelY();
            double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
            last_world_linear_accel_y = curr_world_linear_accel_y;

            if ( ( Math.abs(currentJerkX) > COLLISION_THRESHOLD_DELTA_G ) ||
                    ( Math.abs(currentJerkY) > COLLISION_THRESHOLD_DELTA_G) ) {
                collisionDetected = true;
            }

            setCollisionState( collisionDetected );

        }

        @Override
        public void untimestampedDataReceived(long cur_system_timestamp, Object kind) {

        }

        public void close() {
            this.navx_device.deregisterCallback(this);
        }
    }

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

        DbgLog.msg(String.format("initial_navx_data:yaw=%f,pitch=%f, roll=%f",
                initial_navx_data[0], initial_navx_data[1], initial_navx_data[2]));

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
