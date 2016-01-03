package org.robocracy.ftcrobot.util;
import com.qualcomm.ftccommon.DbgLog;

import java.io.*;

/**
 * Created by pranavb on 12/13/15.
 */
public class FileRW {
    String filePath;
    File file;
    FileReader fileReader;
    FileWriter fileWriter;
    BufferedReader bufferedReader;
    BufferedWriter bufferedWriter;
    public FileRW(String filePath){
        this.filePath = filePath;
        try {
            this.file = new File(filePath);
            file.createNewFile();
            this.fileReader = new FileReader(filePath);
            this.fileWriter = new FileWriter(filePath);
        }
        catch (IOException e){
            DbgLog.error(String.format("An IOException was caught : %s", e.getMessage()));
        }
        this.bufferedReader = new BufferedReader(fileReader);
        this.bufferedWriter = new BufferedWriter(fileWriter);
    }
    public void fileWrite(String data){
        try{
            bufferedWriter.write(data);
            bufferedWriter.newLine();
        }
        catch (IOException e){
            DbgLog.error(String.format("An IOException was caught : %s", e.getMessage()));
        }
    }
    public String fileRead(int lineNum){
        String data = new String();
        try{
            for(int i=0;i<(lineNum-1);i++){
                bufferedReader.readLine();
            }
            data = bufferedReader.readLine();

            bufferedReader.close();
        }
        catch(FileNotFoundException ex) {
            DbgLog.error(String.format(
                    "Unable to open file '%s'", filePath));
        }
        catch (IOException e){
            DbgLog.error(String.format("An IOException was caught : %s", e.getMessage()));
        }


        return data;
    }
    public String getNextLine(){
        String data = new String();
        try{
            data = bufferedReader.readLine();
        }
        catch(FileNotFoundException ex) {
            DbgLog.error(String.format(
                    "Unable to open file '%s'", filePath));
        }
        catch (IOException e){
            DbgLog.error(String.format("An IOException was caught : %s", e.getMessage()));
        }

        return data;
    }
}
