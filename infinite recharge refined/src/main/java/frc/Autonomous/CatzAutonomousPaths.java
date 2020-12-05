package frc.Autonomous;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.Autonomous.CatzAutonomous;
import frc.robot.*;
import frc.Mechanisms.*;

public class CatzAutonomousPaths
{
    static CatzAutonomous auton = new CatzAutonomous();

    public boolean spunUp = false;
    public boolean doneShooting = false;
    public static boolean done;

    public final int DRIVE_DIST_HIGH_SPEED = 16000;
    public final int DRIVE_DIST_MED_SPEED = 10000;
    public final int DRIVE_DIST_LOW_SPEED = 7500;
    public final int TURN_SPEED = 12000;

    public static boolean goalMet;

    public final static double STRAIGHT_BLACKTOP_POWER_PORT_DIST      = -78-1;
    public final static double STRAIGHT_BLACKTOP_PAST_START_LINE_DIST = 130;

    public final static int STRAIGHT_BLACKTOP_MED_SPEED  = 12000;
    public final static int STRAIGHT_BLACKTOP_HIGH_SPEED = 16000;

    public final static double SIDE_BLACKTOP_DRIVE_DIST = 24;
    
    public final static int SIDE_BLACKTOP_DRIVE_SPEED = 12000;

    public final static double DEFAULT_DIST = 60;

    public final static int DEFAULT_SPEED = 12000;
    
    public final static double STRAIGHT_BLUE_PAST_START_LINE_DIST = 0;
    public final static double STRAIGHT_RED_PAST_START_LINE_DIST  = 0;

    public final static int STRAIGHT_BLUE_MED_SPEED  = 12000;
    public final static int STRAIGHT_BLUE_HIGH_SPEED = 16000;

    public final static int STRAIGHT_RED_MED_SPEED  = 12000;
    public final static int STRAIGHT_RED_HIGH_SPEED = 16000;    

    public final static double STRAIGHT_BLUE_POWER_PORT_DIST= 0.0;
    public final static double STRAIGHT_RED_POWER_PORT_DIST = 0.0;

    public final static double SIDE_BLUE_DRIVE_DIST= 0.0;
    public final static double SIDE_RED_DRIVE_DIST = 0.0;

    public final static int STRAIGHT_BLUE_DRIVE_SPEED = 0;
    public final static int STRAIGHT_RED_DRIVE_SPEED  = 0;

    public final static int SIDE_BLUE_DRIVE_SPEED = 0;
    public final static int SIDE_RED_DRIVE_SPEED  = 0;

    public enum AUTO_STATE 
    {
        AS_INIT, AS_DRIVETO, AS_DRIVEBACK, AS_DONE, AS_CHECK_SHOOTER_DONE, AS_WAIT_FOR_SHOOTER_READY;
    }

    public enum AUTO_STATE2 
    {
        AS_INIT, AS_DRIVETO,  AS_DONE;
    }
    public static AUTO_STATE autoState = AUTO_STATE.AS_INIT;
    public static AUTO_STATE2 autoState2 = AUTO_STATE2.AS_INIT;

    public CatzAutonomousPaths() 
    {
        done = false;
    }

    /**
     * 
     * @param position - The path the robot will be taking (e.g. "STRAIGHT")
     */
    public static void monitorAutoState(String position)//, String team) 
    {

        SmartDashboard.putString("Autonomous State", autoState.toString());  //prints the enum text to dashboard; keep track of current sequence TV 3/4/2020
        SmartDashboard.putBoolean("Goal Met", goalMet);

        if (position.equalsIgnoreCase("STRAIGHT"))
        {
            if(!done)
            {
                switch (autoState) 
                {
                    case AS_INIT:
                        auton.setDistanceGoal(STRAIGHT_BLACKTOP_POWER_PORT_DIST, STRAIGHT_BLACKTOP_MED_SPEED);

                        Robot.shooter.setTargetRPM(Robot.shooter.SHOOTER_TARGET_RPM_LO); //TODO LO??
                        System.out.println("A");
                        autoState = AUTO_STATE.AS_DRIVETO;
                    break;

                    case AS_DRIVETO:
                        System.out.println("B1");
                        //System.out.println("drive to, " + goalMet + ", " + Robot.shooter.getShooterReadyState());
                        goalMet = auton.monitorEncoderPosition();
                        if (goalMet) 
                        {
                            System.out.println("B2");
                            autoState = AUTO_STATE.AS_WAIT_FOR_SHOOTER_READY;
                        }
                    break;

                    case AS_WAIT_FOR_SHOOTER_READY:
                        System.out.println("C1");
                        if(Robot.shooter.getShooterReadyState())
                        {
                            System.out.println("C2");
                            Robot.shooter.shoot();
                            autoState = AUTO_STATE.AS_CHECK_SHOOTER_DONE;
                        }
                    break;

		            case AS_CHECK_SHOOTER_DONE:
			            System.out.println("D1");
			            if(Robot.shooter.getShooterDoneState())
			            {
                            System.out.println("D2");
                            auton.setDistanceGoal(STRAIGHT_BLACKTOP_PAST_START_LINE_DIST, STRAIGHT_BLACKTOP_HIGH_SPEED);
			                autoState = AUTO_STATE.AS_DRIVEBACK;
			            }
                    break;

                    case AS_DRIVEBACK:
                        goalMet = auton.monitorEncoderPosition();
                        System.out.println("drive back");
                        if (goalMet) 
                        {
                            System.out.println("E");
                            autoState = AUTO_STATE.AS_DONE;
                        }
                    break;

                    case AS_DONE:
                        System.out.println(position + " - DONE");
                        done = true;
                    break;
            
                }        
            }  
        }
        /*else if(position.equalsIgnoreCase("LEFT") || position.equalsIgnoreCase("RIGHT"))
        {

            if(!done)
            {

                switch (autoState2)
                {

                    case AS_INIT:
                        auton.setDistanceGoal(SIDE_BLACKTOP_DRIVE_DIST, SIDE_BLACKTOP_DRIVE_SPEED);
                        autoState = AUTO_STATE.AS_DRIVETO;
                    break;
                    
                    case AS_DRIVETO:
                        goalMet = auton.monitorEncoderPosition();

                        if(goalMet)
                        {

                            autoState = AUTO_STATE.AS_DONE;

                        }

                    break;
                    
                    case AS_DONE:
                        System.out.println(position + " - DONE");

                        done = true;
                    break;

                }

            }

        }

        else
        {
            if(!done)
            {
                switch (autoState2)
                {

                    case AS_INIT:
                        auton.setDistanceGoal(DEFAULT_DIST, DEFAULT_SPEED);
                        autoState = AUTO_STATE.AS_DRIVETO;
                        break;
                    
                    case AS_DRIVETO:
                        goalMet = auton.monitorEncoderPosition();

                        if(goalMet)
                        {

                            autoState = AUTO_STATE.AS_DONE;

                        }
                        break;
                    
                    case AS_DONE:
                        System.out.println(position + " DONE");
                        done = true;
                        break;
                }
        
            }
        
        }

        if (position.equalsIgnoreCase("BLUE"))
        {

            if (position.equalsIgnoreCase("STRAIGHT"))
            {
                
                if(!done)
                {
                    switch (autoState) 
                    {
                        case AS_INIT:
                            auton.setDistanceGoal(STRAIGHT_BLUE_POWER_PORT_DIST, STRAIGHT_BLUE_DRIVE_SPEED);

                            Robot.shooter.setTargetRPM(Robot.shooter.SHOOTER_TARGET_RPM_LO); //TODO LO??
                            
                            autoState = AUTO_STATE.AS_DRIVETO;
                        break;

                        case AS_DRIVETO:
                            goalMet = auton.monitorEncoderPosition();
                            //System.out.println("drive to, " + goalMet + ", " + Robot.shooter.getShooterReadyState());
                            if (goalMet)// && Robot.shooter.getShooterReadyState()) 
                            {
                                if(Robot.shooter.getShooterReadyState())
                                {
                                    autoState = AUTO_STATE.AS_SHOOT;
                                }
                            }/*
                            else
                            {
                                goalMet = auton.monitorEncoderPosition();
                            }
                        break;

                        case AS_SHOOT:
                            System.out.println("Shoot" + Robot.shooter.getShooterReadyState());
                            if (Robot.shooter.getShooterReadyState() == false)//.shooter.shooterState == Robot.shooter.SHOOTER_STATE_OFF) 
                            {
                                auton.setDistanceGoal(STRAIGHT_BLUE_PAST_START_LINE_DIST, STRAIGHT_BLUE_HIGH_SPEED);
                                autoState = AUTO_STATE.AS_DRIVEBACK;
                            }
                        break;

                        case AS_DRIVEBACK:
                            goalMet = auton.monitorEncoderPosition();
                            System.out.println("drive back");
                            if (goalMet) 
                            {
                                autoState = AUTO_STATE.AS_DONE;
                            }
                        break;

                        case AS_DONE:
                            System.out.println(position + " - DONE");
                            done = true;
                        break;
                
                    }        
                }  
            }
            else if(position.equalsIgnoreCase("LEFT") || position.equalsIgnoreCase("RIGHT"))
            {

                if(!done)
                {

                    switch (autoState)
                    {

                        case AS_INIT:
                            auton.setDistanceGoal(SIDE_BLUE_DRIVE_DIST, SIDE_BLUE_DRIVE_SPEED);
                            autoState = AUTO_STATE.AS_DRIVETO;
                        break;
                        
                        case AS_DRIVETO:
                            goalMet = auton.monitorEncoderPosition();

                            if(goalMet)
                            {

                                autoState = AUTO_STATE.AS_DONE;

                            }

                        break;
                        
                        case AS_DONE:
                            System.out.println(position + " - DONE");

                            done = true;
                        break;

                    }

                }

            }

            else
            {
                if(!done)
                {
                    switch (autoState)
                    {

                        case AS_INIT:
                            auton.setDistanceGoal(DEFAULT_DIST, DEFAULT_SPEED);
                            autoState = AUTO_STATE.AS_DRIVETO;
                            break;
                        
                        case AS_DRIVETO:
                            goalMet = auton.monitorEncoderPosition();

                            if(goalMet)
                            {

                                autoState = AUTO_STATE.AS_DONE;

                            }
                            break;
                        
                        case AS_DONE:
                            System.out.println(position + " DONE");
                            done = true;
                            break;
                    }
            
                }
            
            }
        }

        else if (position.equalsIgnoreCase("RED"))
        {
            if (position.equalsIgnoreCase("STRAIGHT"))
            {
                
                if(!done)
                {
                    switch (autoState) 
                    {
                        case AS_INIT:
                            auton.setDistanceGoal(STRAIGHT_RED_POWER_PORT_DIST, STRAIGHT_RED_DRIVE_SPEED);

                            Robot.shooter.setTargetRPM(Robot.shooter.SHOOTER_TARGET_RPM_LO); //TODO LO??
                            
                            autoState = AUTO_STATE.AS_DRIVETO;
                        break;

                        case AS_DRIVETO:
                            goalMet = auton.monitorEncoderPosition();
                            //System.out.println("drive to, " + goalMet + ", " + Robot.shooter.getShooterReadyState());
                            if (goalMet)// && Robot.shooter.getShooterReadyState()) 
                            {
                                if(Robot.shooter.getShooterReadyState())
                                {
                                    autoState = AUTO_STATE.AS_SHOOT;
                                }
                            }/*
                            else
                            {
                                goalMet = auton.monitorEncoderPosition();
                            }
                        break;

                        case AS_SHOOT:
                            System.out.println("Shoot" + Robot.shooter.getShooterReadyState());
                            if (Robot.shooter.getShooterReadyState() == false)//.shooter.shooterState == Robot.shooter.SHOOTER_STATE_OFF) 
                            {
                                auton.setDistanceGoal(STRAIGHT_RED_PAST_START_LINE_DIST, STRAIGHT_RED_HIGH_SPEED);
                                autoState = AUTO_STATE.AS_DRIVEBACK;
                            }
                        break;

                        case AS_DRIVEBACK:
                            goalMet = auton.monitorEncoderPosition();
                            System.out.println("drive back");
                            if (goalMet) 
                            {
                                autoState = AUTO_STATE.AS_DONE;
                            }
                        break;

                        case AS_DONE:
                            System.out.println(position + " - DONE");
                            done = true;
                        break;
                
                    }        
                }  
            }
            else if(position.equalsIgnoreCase("LEFT") || position.equalsIgnoreCase("RIGHT"))
            {

                if(!done)
                {

                    switch (autoState)
                    {

                        case AS_INIT:
                            auton.setDistanceGoal(SIDE_RED_DRIVE_DIST, SIDE_RED_DRIVE_SPEED);
                            autoState = AUTO_STATE.AS_DRIVETO;
                        break;
                        
                        case AS_DRIVETO:
                            goalMet = auton.monitorEncoderPosition();

                            if(goalMet)
                            {

                                autoState = AUTO_STATE.AS_DONE;

                            }

                        break;
                        
                        case AS_DONE:
                            System.out.println(position + " - DONE");

                            done = true;
                        break;

                    }

                }

            }

            else
            {
                if(!done)
                {
                    switch (autoState)
                    {

                        case AS_INIT:
                            auton.setDistanceGoal(DEFAULT_DIST, DEFAULT_SPEED);
                            autoState = AUTO_STATE.AS_DRIVETO;
                            break;
                        
                        case AS_DRIVETO:
                            goalMet = auton.monitorEncoderPosition();

                            if(goalMet)
                            {

                                autoState = AUTO_STATE.AS_DONE;

                            }
                            break;
                        
                        case AS_DONE:
                            System.out.println(position + " DONE");
                            done = true;
                            break;
                    }
            
                }
            
            }
        }*/
    }

}