package org.robocracy.ftcrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriveSystem.AWDOmniWheelDS;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;


/**
 * Top level class in hierarchy. Represents an {@code FTCRobot} with
 *     main {@link FTCRobot#runRobotTeleop()} methods
 * @author Team Robocracy
 * {@docRoot}
 */
public class FTCRobot {
    LinearOpMode curOpmode;
    public AWDMecanumDS mecanumDriveSys;
    public AWDOmniWheelDS omniDriveSys;

    DriverStation drvrStation;
    public FTCRobot(LinearOpMode curOpmode, String dsType) throws InterruptedException {
        this.curOpmode = curOpmode;

        if (dsType.matches("MecanumDS")) {
            this.drvrStation = new DriverStation(curOpmode, this, 8);
            this.mecanumDriveSys = new AWDMecanumDS(curOpmode, this);
            this.omniDriveSys = null;
        }
        else {
            this.drvrStation = new DriverStation(curOpmode, this, 4);
            this.omniDriveSys = new AWDOmniWheelDS(curOpmode, this);
            this.mecanumDriveSys = null;
        }
    }

    /**
     * Runs Teleop mode by {@link DriverStation#getNextCommand()} for getting gamepad values.
     * @throws InterruptedException
     */
    public  void  runRobotTeleop() throws InterruptedException {
        DriverCommand driverCommand;
        while(curOpmode.opModeIsActive()){
            driverCommand = drvrStation.getNextCommand();
            if (this.mecanumDriveSys != null) {
                this.mecanumDriveSys.applyCmd(driverCommand);
            } else if (this.omniDriveSys != null) {
                this.omniDriveSys.applyCmd(driverCommand);
            }

            // Wait for one hardware cycle for the setPower(0) to take effect.
            this.curOpmode.waitForNextHardwareCycle();

        }
    }

    public void close() {
        if (this.mecanumDriveSys != null) {
            this.mecanumDriveSys.close();
        } else if (this.omniDriveSys != null) {
            // nothing to do
        }
    }
}
