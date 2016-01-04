package org.robocracy.ftcrobot;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;
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
    DcMotor liftDirectionMotor;

    public LinearLift(FTCRobot robot, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.liftAngleMotor = curOpMode.hardwareMap.dcMotor.get("liftAngleMotor");
        this.liftDirectionMotor = curOpMode.hardwareMap.dcMotor.get("liftDirectionMotor");
    }

    /**
     * Applies power to lift motors based on value in {@code double direction, angle} set in {@link DriverStation#getNextDrivesysCmd()}.
     * @param driverCommand {@link DriverCommand} object with values.
     */
    public void applyCmd(DriverCommand driverCommand){
        liftDirectionMotor.setPower(driverCommand.linliftcmd.direction);
        liftAngleMotor.setPower(driverCommand.linliftcmd.angle);
    }
}
