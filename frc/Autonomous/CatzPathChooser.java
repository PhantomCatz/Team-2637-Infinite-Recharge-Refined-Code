package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CatzPathChooser
{

    public static boolean checkBoxL;
    public static boolean checkBoxM;
    public static boolean checkBoxR;

    public static void choosePath()
    {
        checkBoxL = SmartDashboard.getBoolean("Position Left", false);
        checkBoxM = SmartDashboard.getBoolean("Position Middle", false);
        checkBoxR = SmartDashboard.getBoolean("Position Right", false);

        //red and blue sides symmetrical, no check box for color required
        if(checkBoxL == true)
        {

            CatzAutonomousPaths.monitorAutoState("LEFT");

        }
        else if(checkBoxM == true)
        {

            CatzAutonomousPaths.monitorAutoState("STRAIGHT");

        }
        else if(checkBoxR == true)
        {

            CatzAutonomousPaths.monitorAutoState("RIGHT");

        }
        else
        {
            System.out.println("no path was chosen - performing default");

            //drive past starting line and stop

        }


    }


}