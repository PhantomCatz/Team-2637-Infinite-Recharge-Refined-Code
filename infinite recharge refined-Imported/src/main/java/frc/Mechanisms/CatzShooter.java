package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.CatzAutonomousPaths;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class CatzShooter 
{   
    public WPI_TalonSRX shtrMtrCtrlA;
    public WPI_VictorSPX shtrMtrCtrlB;

    private final int SHTR_MC_ID_A = 40; 
    private final int SHTR_MC_ID_B = 41; 
    
    final double COUNTS_PER_REVOLUTION      = 4096.0;
    final double SEC_TO_MIN                 = 60.0;
    final double ENCODER_SAMPLE_RATE_MSEC   = 100.0;
    final double ENCODER_SAMPLE_PERIOD_MSEC = (1.0 / ENCODER_SAMPLE_RATE_MSEC);
    final double MSEC_TO_SEC                = 1000.0;
    final double FLYWHEEL_GEAR_REDUCTION    = 3.0;

    final double CONV_QUAD_VELOCITY_TO_RPM = ( ((ENCODER_SAMPLE_PERIOD_MSEC * MSEC_TO_SEC * SEC_TO_MIN) / COUNTS_PER_REVOLUTION)); //converts velocity to RPM

    public static final int SHOOTER_STATE_OFF                 = 0;
    public static final int SHOOTER_STATE_RAMPING             = 1;
    public static final int SHOOTER_STATE_SET_SPEED           = 2;
    public static final int SHOOTER_STATE_READY               = 3;
    public static final int SHOOTER_STATE_START_SHOOTING      = 4;
    public static final int SHOOTER_STATE_WAIT_FOR_SHOOT_DONE = 5;
    
    public final double SHOOTER_RPM_START_OFFSET =  250.0;
    public final double SHOOTER_TARGET_RPM_LO    = (4700.0 - SHOOTER_RPM_START_OFFSET); //4400
    public final double SHOOTER_TARGET_RPM_MD    = (5000.0 - SHOOTER_RPM_START_OFFSET);
    public final double SHOOTER_TARGET_RPM_HI    = (6000.0 - SHOOTER_RPM_START_OFFSET);

    final double SHOOTER_BANG_BANG_MAX_RPM_OFFSET = 5.0; 
    final double SHOOTER_BANG_BANG_MIN_RPM_OFFSET = 5.0;

    final double SHOOTER_RAMP_RPM_OFFSET = 200.0;

    final double SHOOTER_OFF_POWER   =  0.0;
    final double SHOOTER_RAMP_POWER  = -1.0;
    final double SHOOTER_SHOOT_POWER = -1.0;

    final int NUM_OF_DATA_SAMPLES_TO_AVERAGE = 5;

    final double SHOOTER_THREAD_PERIOD           = 0.040;
    final long   SHOOTER_THREAD_PERIOD_MS        = 40;
    final double SHOOTER_RAMP_TIMEOUT_SEC        = 4.000;  //TBD-TEST put back to 4.0
    final double INDEXER_SHOOT_TIME_SEC          = 1.60;
    final double SHOOTER_AVG_VEL_SAMPLE_TIME_SEC = 0.100;

    public double targetRPM          = 0.0;
    public double targetRPMThreshold = 0.0;
    public double shooterPower       = 0.0;
    public double minPower           = 0.0;
    public double maxPower           = 0.0;

    public boolean logTestData = true;
    
    private Thread shooterThread;

    public static int shooterState = SHOOTER_STATE_OFF;

    private int indexerShootStateCountLimit  = 0;
    private int rampStateCountLimit          = 0;
    private int indexerShootStateCount       = 0;
    private int rampStateCount               = 0;
    private int samplingVelocityCount        = 0;
    private int samplingVelocityCountLimit   = 0;

    private boolean shooterIsReady = false;
    public  boolean shooterIsDone  = true;
    double avgVelocity             = 0.0;

    public boolean inAutonomous;


    public CatzShooter() //constructor
    {   
        shtrMtrCtrlA = new WPI_TalonSRX(SHTR_MC_ID_A);
        shtrMtrCtrlB = new WPI_VictorSPX(SHTR_MC_ID_B); //intialize motor controllers

        //Reset configuration
        shtrMtrCtrlA.configFactoryDefault();
        shtrMtrCtrlB.configFactoryDefault();

        //Set MC B to follow MC A
       shtrMtrCtrlB.follow(shtrMtrCtrlA);

        //Configure feedback device for PID loop
        shtrMtrCtrlA.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

        shtrMtrCtrlA.getSensorCollection().setQuadraturePosition(0,100); 

        //Set MC's to coast mode
        shtrMtrCtrlA.setNeutralMode(NeutralMode.Coast);
        shtrMtrCtrlB.setNeutralMode(NeutralMode.Coast);

        //Configure PID constants
        shtrMtrCtrlA.config_kP(0, 0.005);
        shtrMtrCtrlA.config_kI(0, 0.0);
        shtrMtrCtrlA.config_kD(0, 0.0);
        shtrMtrCtrlA.config_kF(0, 0.008); //shtrMtrCtrlA.config_kF(0, 0.008);
        shtrMtrCtrlA.config_IntegralZone(0, 0);

         //limits how long the shooter runs so it doesn't go too long (limiter)
        indexerShootStateCountLimit = (int)Math.round( (INDEXER_SHOOT_TIME_SEC / SHOOTER_THREAD_PERIOD) + 0.5 ); 
        //equation which limits how long the ramp up goes (don't want it to go too much/fast)
        rampStateCountLimit         = (int)Math.round( (SHOOTER_RAMP_TIMEOUT_SEC / SHOOTER_THREAD_PERIOD) + 0.5);
        //equation which determines the time between each sample (flywheel velocity)
        samplingVelocityCountLimit  = (int)Math.round( (SHOOTER_AVG_VEL_SAMPLE_TIME_SEC / SHOOTER_THREAD_PERIOD) + 0.5);
        
        setShooterVelocity();
        shooterOff();
    }

    public void setTargetVelocity(double targetVelocity)  //TBD //not using
    {
        shtrMtrCtrlA.set(ControlMode.Velocity, targetVelocity);
       //shtrMtrCtrlA.set(ControlMode.PercentOutput, targetVelocity);
    }

    public void shooterFlyWheelDisable()// not being used
    {
        shtrMtrCtrlA.set(0);
    }

    public double getFlywheelShaftPosition() //encoder counts
    {
        return shtrMtrCtrlA.getSensorCollection().getQuadraturePosition();
    }
    public double getFlywheelShaftVelocity() //RPM
    {
        return (Math.abs((double) shtrMtrCtrlA.getSensorCollection().getQuadratureVelocity()) * CONV_QUAD_VELOCITY_TO_RPM); 
    }

    public void setTargetRPM(double velocity) // Sets the RPM (determined by the button pressed on controller)
    {
        targetRPM = velocity;
    }

    public void shoot() // when a button is pressed on controller, it will cause the indexer to move and launch balls. sets state to ready
    {
        if(shooterState == SHOOTER_STATE_READY)
        {
            indexerShootStateCount = 0;
            shooterState = SHOOTER_STATE_START_SHOOTING;
        }
    }

    public void shooterOff() // turns shooter off , sets the shooter state to off
    {
        targetRPM      = 0.0;
        shooterIsReady = false;
	    shooterIsDone  = true;
        shooterState   = SHOOTER_STATE_OFF;
        shooterPower   = SHOOTER_OFF_POWER;
        shtrMtrCtrlA.set(shooterPower);
        Robot.indexer.setShooterIsRunning(false);
        Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0);
    
    }

    public void bangBang(double minRPM, double maxRPM, double flywheelShaftVelocity) // bangbang method
    {
        if (flywheelShaftVelocity > maxRPM)
        {
            shooterPower = minPower; 
        }
        else if(flywheelShaftVelocity < minRPM) 
        {
            shooterPower = maxPower;
        }
        shtrMtrCtrlA.set(shooterPower);
    }

    public void setShooterVelocity() //will make shooter run and etc
    {

        shooterThread = new Thread(() -> //start of thread
        {
            double flywheelShaftVelocity    = -1.0;
            double minRPM                   = 0.0;
            double maxRPM                   = 0.0;
            double shootTime                = 0.0;
            double[] velocityData           = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double sumOfVelocityData        = 0.0;
            int velocityDataIndex           = 0;
            boolean readyToCalculateAverage = false;
            boolean rumbleSet               = false;

            while(true)
            {
                shootTime = Robot.dataCollectionTimer.get();
                flywheelShaftVelocity = getFlywheelShaftVelocity();

                switch (shooterState)
                {
                    case SHOOTER_STATE_OFF: //when there is no targetRPM (basically when no button is pressed) will be shooter most of the time
                        shooterPower = SHOOTER_OFF_POWER;

                        if(targetRPM > 0.0)
                        {
                            indexerShootStateCount  = 0;
                            rampStateCount          = 0;
                            samplingVelocityCount   = 0;
                            sumOfVelocityData       = 0.0;
                            velocityDataIndex       = 0;
                            readyToCalculateAverage = false;
                            shooterState            = SHOOTER_STATE_RAMPING;
                            targetRPMThreshold      = targetRPM - SHOOTER_RAMP_RPM_OFFSET;
                            minRPM                  = targetRPM - SHOOTER_BANG_BANG_MIN_RPM_OFFSET;
                            maxRPM                  = targetRPM + SHOOTER_BANG_BANG_MAX_RPM_OFFSET;
                            shooterPower            = SHOOTER_RAMP_POWER;
                            rumbleSet               = false;
            			    shooterIsDone           = false;

                            getBangBangPower();
                            shtrMtrCtrlA.set(shooterPower);

                            for(int i = 0; i < NUM_OF_DATA_SAMPLES_TO_AVERAGE; i++ )
                            {
                                velocityData[i] = 0.0;
                            }

                            System.out.println("T1: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);
                        }

                    break;

                    case SHOOTER_STATE_RAMPING: // once targetRPM is given, velocity ramps up as fast as possible to reach targetRPM
                        Robot.indexer.setShooterRamping(true);
                        if(flywheelShaftVelocity > targetRPMThreshold)
                        {
                            shooterState = SHOOTER_STATE_SET_SPEED;
                            shooterPower = maxPower;
                            shtrMtrCtrlA.set(shooterPower);
                            System.out.println("T2: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower );

                        }
                        rampStateCount++;
                        if(rampStateCount > rampStateCountLimit)
                        {
                            shooterOff();
                        }
                        break;

                    case SHOOTER_STATE_SET_SPEED: // making bang bang work. adds up RPM (prerequisite for bang bang, checks average of )
                        samplingVelocityCount++;
                        if(samplingVelocityCount > samplingVelocityCountLimit)
                        {
                            samplingVelocityCount = 0;
                            velocityData[velocityDataIndex++ ] = flywheelShaftVelocity;
                            System.out.print("S");
                            Robot.indexer.setShooterRamping(false);
                            if(velocityDataIndex == NUM_OF_DATA_SAMPLES_TO_AVERAGE)
                            {
                                velocityDataIndex = 0;
                                readyToCalculateAverage = true;
                            }

                            if(readyToCalculateAverage == true)
                            {
                                
                                for(int i = 0; i < NUM_OF_DATA_SAMPLES_TO_AVERAGE; i++ )
                                {  
                                    sumOfVelocityData = sumOfVelocityData + velocityData[i];
                                }
                        
                                avgVelocity = sumOfVelocityData / NUM_OF_DATA_SAMPLES_TO_AVERAGE;
                                sumOfVelocityData = 0.0;
                                System.out.println("AD: " + avgVelocity);
                            }

                            if(avgVelocity > minRPM && avgVelocity < maxRPM)
                            {
                                shooterState = SHOOTER_STATE_READY;
                            }
                        }

                        bangBang(minRPM, maxRPM, flywheelShaftVelocity);

                        System.out.println("T3: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);

                    break;

                    case SHOOTER_STATE_READY:// makes the controller vibrate so that aux driver knows to shoot
                        shooterIsReady = true;
                        System.out.println("SSR");
                        bangBang(minRPM, maxRPM, flywheelShaftVelocity);   

                        if(inAutonomous == true)
                        { 
                        }
                        else 
                        {
                            if(rumbleSet == false)
                            {
                                Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 1);
                                rumbleSet = true;
                            } 
                        }
                    break;

                    case SHOOTER_STATE_START_SHOOTING: 
                        shooterPower = SHOOTER_SHOOT_POWER;
                        shtrMtrCtrlA.set(shooterPower);    
                        System.out.println("TS1: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);

                        if(flywheelShaftVelocity > targetRPM + SHOOTER_RPM_START_OFFSET)
                        {
                            Robot.indexer.indexerStart(); 
                            shooterState = SHOOTER_STATE_WAIT_FOR_SHOOT_DONE;
                            System.out.println("TS2: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);
                        }
                    break;

                    case SHOOTER_STATE_WAIT_FOR_SHOOT_DONE: //will count for a certain amount of time until it switches the shooter off and sets state to OFF
                        System.out.println("TS3: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);
                        indexerShootStateCount++;
                        if(indexerShootStateCount > indexerShootStateCountLimit)
                        {
                            shooterOff();
                            Robot.indexer.indexerStop(); 
                            Robot.indexer.setShooterIsRunning(false);
                            System.out.println("TS4: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);
                        }
                
                    break;

                    default:  //default code when there is nothing going on 
                        shooterOff();
                    break;
            }        
            Timer.delay(SHOOTER_THREAD_PERIOD);
        }
    }); //end of thread
        shooterThread.start();
    }
    //testing power and resulting rpm
/*
    *  power 0.62  6100 rpm 
    *  power 0.55  5400 rpm
    *  power 0.54  5300 rpm
    *  power 0.52  5100 rpm
    *  power 0.50  4700 rpm
    *  power 0.48  4500 rpm
    *  power 0.45  4200 rpm
    */

    public void autonomousOn()
    {
        inAutonomous = true;
    }

    public void autonomousOff()
    {
        inAutonomous = false;
    }
    

    public boolean getShooterReadyState()
    {
        return shooterIsReady;
    }

    public boolean getShooterDoneState()
    {
	    return shooterIsDone;
    }

    public void getBangBangPower() //determines max and min power based on the velocity chosen
    {
       double power =  (targetRPM / 10000.0) + 0.04; //+0.05    
       minPower = -(power - 0.05);
       maxPower = -(power + 0.05);
        
     /*   if(targetRPM < 4000.0)
        {
            maxPower = -0.50;
            minPower = -0.40;
        }
        else if(targetRPM >= 4000.0 && targetRPM < 4500.0)
        {
            maxPower = -0.50;
            minPower = -0.40;
        }
        else if(targetRPM >= 4500.0 && targetRPM < 5000.0)
        {
            maxPower = -0.50;
            minPower = -0.45;
        }
        else if(targetRPM >= 5000.0 && targetRPM < 5500.0)
        {
            maxPower = -0.55;
            minPower = -0.45;
        }
        else if (targetRPM >= 5500.0 && targetRPM <6000.0)
        {
            maxPower = -0.60;
            minPower = -0.50;
        }
        else if (targetRPM >= 6000.0)
        {
            maxPower = -0.90;
            minPower = -0.45;
        } */
    }

    public void debugSmartDashboard()
    { 
        //smart dashboard only to be used during testing(before comp)
        SmartDashboard.putNumber("RPM",             getFlywheelShaftVelocity() );
        SmartDashboard.putNumber("Power",           shooterPower);
        SmartDashboard.putNumber("Target Velocity", targetRPM);
        SmartDashboard.putNumber("ENC Position",    getFlywheelShaftPosition());
        SmartDashboard.putNumber("Average rpm",  avgVelocity);
    }
    public void smartdashboard() //what will be used during comp
    {
        SmartDashboard.putBoolean("Shooter ready", shooterIsReady); 
    }
}