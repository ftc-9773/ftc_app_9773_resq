package org.robocracy.ftcrobot.util;
import com.qualcomm.ftccommon.DbgLog;

import java.io.*;

/**
 * @author Team Robocracy
 * Enables file read/write capabilities throughout all classes and methods.
 */
public class FileRW {
    String filePath;
    File file;
    FileReader fileReader;
    FileWriter fileWriter;
    BufferedReader bufferedReader;
    BufferedWriter bufferedWriter;
    public FileRW(String filePath, boolean isWrite){
        this.filePath = filePath;
        try {
            this.file = new File(filePath);
            if(isWrite) {
                file.createNewFile();
                this.fileWriter = new FileWriter(filePath);
            }
            else {
                this.fileReader = new FileReader(filePath);
            }
        }
        catch (IOException e){
            DbgLog.error(String.format("An IOException was caught : %s", e.getMessage()));
        }
        if(isWrite){
            this.bufferedWriter = new BufferedWriter(fileWriter);
        }
        else {
            this.bufferedReader = new BufferedReader(fileReader);
        }
    }

    /**
     * Writes {@code String} of data to the object's {@code filePath}.
     * @param data data to be written
     */
    public void fileWrite(String data){
        try{
            bufferedWriter.write(data);
            bufferedWriter.newLine();
        }
        catch (IOException e){
            DbgLog.error(String.format("An IOException was caught : %s", e.getMessage()));
        }
    }

    /**
     * Reads file at line specified in object's {@code filePath}
     * @param lineNum line to be read
     * @return data at line
     */
    public String fileRead(int lineNum){
        String data = null;
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

    /**
     * Gets next line of file.
     * @return next line of data
     */
    public String getNextLine(){
        String data = null;
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

    public void close() throws IOException {
        if (this.fileWriter != null) {
            this.fileWriter.flush(); // Write the contents in memory to the disk
//            this.fileWriter.close();
        }

        if (this.fileReader != null) {
            this.fileReader.close();
        }
    }
}
