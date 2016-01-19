package org.robocracy.ftcrobot;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
/**
 * @author Team Robocracy
 *
 * Operates Linear Lift on robot.
 */
public class LinearLift {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor liftAngleMotor;
    DcMotor liftArmLengthMotor;
    boolean liftAngleMotorAvailable = false;
    boolean liftArmLengthMotorAvailable = false;

    public LinearLift(FTCRobot robot, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            this.liftAngleMotor = curOpMode.hardwareMap.dcMotor.get("liftAngleMotor");
            liftAngleMotorAvailable = true;
        }
        catch(Exception e){
            DbgLog.error(String.format("%s . Device skipped", e.getMessage()));
        }
        try {
            this.liftArmLengthMotor = curOpMode.hardwareMap.dcMotor.get("liftArmLengthMotor");
            liftArmLengthMotorAvailable = true;
        }
        catch(Exception e){
            DbgLog.error(String.format("%s . Device skipped", e.getMessage()));
        }
    }

    /**
     * Applies power to lift motors based on value in {@code double direction, angle} set in {@link DriverStation#getNextDrivesysCmd()}.
     * @param driverCommand {@link DriverCommand} object with values.
     */
    public void applyCmd(DriverCommand driverCommand){
        if (liftArmLengthMotorAvailable) {
            liftArmLengthMotor.setPower(driverCommand.linliftcmd.armLength);
        }
        if (liftAngleMotorAvailable) {
            liftAngleMotor.setPower(driverCommand.linliftcmd.angle);
        }
    }
}
