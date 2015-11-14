package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/*import com.qualcomm.robotcore.eventloop.opmode.OpMode;*/
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.opmodes.DriveSystem;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by pranavb on 11/11/15.
 */
public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("fMotorL");
        DcMotor frontRight = hardwareMap.dcMotor.get("fMotorR");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rMotorL");
        DcMotor rearRight = hardwareMap.dcMotor.get("rMotorR");
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        DriveSystem mecanumDriveSystem = new DriveSystem(frontLeft, frontRight, rearLeft, rearRight);

        waitOneFullHardwareCycle();

        waitForStart();

/*        frontLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rearLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rearRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
*/
        frontLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        frontRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rearLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rearRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);



        waitOneFullHardwareCycle();

        double counts = mecanumDriveSystem.mecanumWheelAutoDrive(60, 0.5);

        waitOneFullHardwareCycle();
        /*mecanumDriveSystem.mecanumWheelDrive(0, 0, 0, 1);
        sleep(2000);
        mecanumDriveSystem.mecanumWheelDrive(0, 0, 0, 0);*/

        while(opModeIsActive()){

            telemetry.addData("Counts to move: ", counts);
            telemetry.addData("front left counts: ", frontLeft.getCurrentPosition());
            telemetry.addData("front right counts: ", frontRight.getCurrentPosition());
            telemetry.addData("rear left counts: ", rearLeft.getCurrentPosition());
            telemetry.addData("rear right counts: ", rearRight.getCurrentPosition());
        }
    }
}
