package org.robocracy.ftcrobot.util;

import java.io.File;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.DocumentBuilder;

import org.w3c.dom.Document;
import org.w3c.dom.NodeList;
import org.w3c.dom.Node;
import org.w3c.dom.Element;

/**
 * Created by pb8xe_000 on 3/5/2016.
 */
public class ConfigParser {
    public String[][] parse(String fileName) throws Exception {
        String[][] robot = new String[1][];
        File inputFile = new File(fileName);
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        Document doc = dBuilder.parse(inputFile);
        doc.getDocumentElement().normalize();
        NodeList partList = doc.getElementsByTagName("Part");
        String[] part = new String[5];
        String[][] parts = new String[partList.getLength()][];
        for (int i = 0; i < partList.getLength(); i++) {
            Node nNode = partList.item(i);
            if (nNode.getNodeType() == Node.ELEMENT_NODE) {
                Element eElement = (Element) nNode;
                part[0] = eElement.getAttribute("name");
                part[1] = eElement.getAttribute("motor-type");
                if(part[1] == "dcmotor") {
                    part[2] = eElement.getAttribute("motor-min");
                    part[3] = eElement.getAttribute("motor-max");
                    part[4] = eElement.getAttribute("encoders");
                }
                else if(part[1] == "servo"){
                    part[2] = eElement.getAttribute("low-position");
                    part[3] = eElement.getAttribute("high-position");
                    part[4] = eElement.getAttribute("reverse");
                }
                part[5] = eElement.getAttribute("gamepad-buttons");

                parts[i] = part;
            }
        }
        NodeList sensorList = doc.getElementsByTagName("Sensor");
        String[] sensor = new String[2];
        String[][] sensors = new String[sensorList.getLength()][];
        for (int i = 0; i < sensorList.getLength(); i++){
            Node nNode = sensorList.item(i);
            if(nNode.getNodeType() == Node.ELEMENT_NODE){

            }
        }

        return robot;
    }
}
