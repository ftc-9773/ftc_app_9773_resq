package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by pranavb on 11/28/15.
 */
public class rliTest extends OpMode {
    DeviceInterfaceModule dim;
    OpticalDistanceSensor opd;

    @Override
    public void init(){
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        opd = hardwareMap.opticalDistanceSensor.get("opd");
    }

    @Override
    public void loop() {
        DbgLog.msg(String.format("rli: %f", opd.getLightDetected()));
    }
}
