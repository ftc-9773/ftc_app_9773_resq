package org.robocracy.ftcrobot.DriveSystem;

import org.json.JSONObject;
import org.json.JSONArray;
import org.json.JSONException;

import java.util.List;

/**
 * Created by michaelzhou on 6/5/16.
 */
public class JSONParser {

    JSONArray obj = null;
    public JSONParser() {
        try {
            obj = new JSONArray("init.json");
            finalReturn();
        }
        catch (JSONException e){
            e.printStackTrace();
        }
    }

    public JSONArray finalReturn() throws JSONException{
            for (int i = 0; i < obj.length(); i++) {
                JSONObject object = obj.getJSONObject(i);

                String motorId = object.getString("motorId");
                String motorType = object.getString("motorType");
                int motorMaxRPM = object.getInt("motorMaxRPM");

                String name = object.getString("name");
                String type = object.getString("type");
                String wheelType = object.getString("wheelType");

                String[] motors = new String[2];
                int[] powerRange = new int[2];
                String[] gamepadButtons = new String[2];

                for (int j = 0; j < 2; j++) {
                    motors[j] = object.getString("motors");
                    powerRange[j] = object.getInt("powerRange");
                    gamepadButtons[j] = object.getString("gamepadButtons");
                }
                String controlType = object.getString("controlType");

               // finalReturn();
            }

        return obj;

    }
}
