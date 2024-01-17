package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Queue;
import frc.robot.Constants.DebugSettings;


public class Debug {
    private String debugFileName;
    
    public Debug(String filename){
        if(DebugSettings.enableLogging){
            debugFileName = "media/sda1/" + filename;
            try {
                FileWriter debugFileDriveSubsystem = new FileWriter(debugFileName);
                debugFileDriveSubsystem.write("");
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public void logln(String logInfo){
        this.log(logInfo + '\n');
    }
    public void log(String logInfo){
        if(DebugSettings.enableLogging){
            try {
                FileWriter debugFileDriveSubsystem = new FileWriter(debugFileName, true);
                debugFileDriveSubsystem.write(logInfo);
                debugFileDriveSubsystem.close();

            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } 
        }
    }


    

}
