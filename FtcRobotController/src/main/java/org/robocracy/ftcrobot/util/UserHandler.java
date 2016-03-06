package org.robocracy.ftcrobot.util;

import java.util.jar.Attributes;

import jdk.internal.org.xml.sax.SAXException;
import jdk.internal.org.xml.sax.helpers.DefaultHandler;

/**
 * Created by pb8xe_000 on 3/5/2016.
 */
public class UserHandler extends DefaultHandler {
    @Override
    public void startElement(String uri, String localName, String tagName, Attributes attributes) throws SAXException{
        if(tagName.equalsIgnoreCase("drive-system")){
            String wheelNum = attributes.getValue("wheels");
            String wheelType = attributes.getValue("type");
            String drivesysGamepad = attributes.getValue("gamepad");
            String linearMvmtInput = attributes.getValue("linear-movement-button");
            String rotationalMvmtInput = attributes.getValue("spin-button");
        }
        else if(tagName.equalsIgnoreCase("part")){
            String partName = attributes.getValue("name");
            String motorType = attributes.getValue("motor-type");
            String servoPositions = attributes.getValue("positions");
            String motorPowerRange = attributes.getValue("power-range");
            String partGamepad = attributes.getValue("gamepad");
            String buttons = attributes.getValue("buttons");
        }
        else if(tagName.equalsIgnoreCase("sensor")){
            String sensorName = attributes.getValue("name");
            String sensorType = attributes.getValue("type");
            String sensorPort = attributes.getValue("port");
        }
    }
}
