package frc.Mechanisms;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class CatzIndexer 
{
    //Switches and Sensors
    private final int INDEXER_ENTRANCE_SWITCH_DIO_PORT = 3;
    private final int INDEXER_EXIT_SWITCH_DIO_PORT     = 4;
    private final int BALL_SENSOR_INPUT_DIO_PORT       = 5;
    private final int BALL_SENSOR_OUTPUT_DIO_PORT      = 6;

    private final double BELT_SPEED_LOAD         = 0.4;
    private final double BELT_SPEED_SHOOT        = 0.90; //0.6
    private final double BALL_IN_RANGE_THRESHOLD = 6.0;     //If Ultrasonic detects ball below this range (inches), motor is turned on
    public        double sensorRange             = 0.0;    //Ultrasonic output value

    public DigitalInput indexerEntranceSwitch;
    public DigitalInput indexerExitSwitch;
    public Ultrasonic   ballSensor;

    private final boolean BALL_PRESENT     = false; //Bump switch pressed reads false, not pressed reads true
    private final boolean BALL_NOT_PRESENT = true;

    private boolean indexerEntranceSwitchState = BALL_NOT_PRESENT; 
    private boolean indexerExitSwitchState     = BALL_NOT_PRESENT;  

    //Motor and Motor Controller
    private CANSparkMax  indexerMtrCtrl;

    public  final int INDEXER_MC_PDP_PORT      = 10;
    private final int INDEXER_MC_CAN_ID        = 20;
    private final int INDEXER_MC_CURRENT_LIMIT = 60; //TBD verify no damage for duration of match

    //Ball Management
    public final int MAX_NUM_BALLS = 5;     //if this value is reached, indexer is deactivated.
    public       int ballCount     = 0;   

    public boolean transferingBallToIndexer = false;    //true when motor is active (moving ball from intexer into indexer)

    private boolean shooterRamping          = false;
    private boolean shooterRunning          = false;
    private boolean reachedMaxCapacity      = false;
    private boolean reverseIndexerState     = false;
    private boolean prevReverseIndexerState = false;
    private boolean testMode                = false;

    private Thread indexerThread;

    public CatzIndexer()
    {
        indexerEntranceSwitch = new DigitalInput(INDEXER_ENTRANCE_SWITCH_DIO_PORT);
        indexerExitSwitch     = new DigitalInput(INDEXER_EXIT_SWITCH_DIO_PORT);

        ballSensor = new Ultrasonic(BALL_SENSOR_INPUT_DIO_PORT,BALL_SENSOR_OUTPUT_DIO_PORT);//(input,output)
        ballSensor.setAutomaticMode(true);

        indexerMtrCtrl = new CANSparkMax(INDEXER_MC_CAN_ID, MotorType.kBrushless);

        indexerMtrCtrl.restoreFactoryDefaults();
        indexerMtrCtrl.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrl.setSmartCurrentLimit(INDEXER_MC_CURRENT_LIMIT);
        indexerMtrCtrl.set(0.0);

        transferingBallToIndexer = false;
    }

    public void startIndexerThread()
    {
        indexerThread = new Thread(() ->
        {
            while(true)
            {
                indexerEntranceSwitchState = indexerEntranceSwitch.get();
                indexerExitSwitchState     = indexerExitSwitch.get();
        
                if(shooterRunning)
                {
                    ballCount = 0; //this is assuming that when we run the shooter, it will shoot all balls from the indexer
                    indexerMtrCtrl.set(BELT_SPEED_SHOOT);
                    //System.out.println("shooter running");
                }
                else if(shooterRamping)
                {
                    if(indexerExitSwitchState == BALL_NOT_PRESENT)
                    {
                        //indexerMtrCtrl.set(BELT_SPEED_LOAD);
                        System.out.println("IA");
                    }
                    else
                    {
                        indexerMtrCtrl.set(0.0);
                        System.out.println("IB");
                    }
                }
                else 
                {
                    if(reverseIndexerState == true)
                    {
                        prevReverseIndexerState = true;
                        indexerMtrCtrl.set(-BELT_SPEED_LOAD);
                        //System.out.println("reverse indexer state");
                    }
                    else 
                    {
                        if(prevReverseIndexerState == true)
                        {
                            prevReverseIndexerState = false;
                            indexerMtrCtrl.set(0.0);
                            //System.out.println("prev reverse indexer state");
                        }

                        if(indexerExitSwitchState == BALL_PRESENT)
                        {
                            reachedMaxCapacity = true;  //marks indexer as "full"
                            indexerMtrCtrl.set(0.0);
                            //System.out.println("indexer exit switch");
                        }
                        else  
                        {   
                            sensorRange = getUltraSonicSensorReading();

                            if(sensorRange < BALL_IN_RANGE_THRESHOLD)
                            { 
                                transferingBallToIndexer = true;
                                indexerMtrCtrl.set(BELT_SPEED_LOAD);
                                System.out.println("INDEXER ON");

                            }
                            else 
                            {
                                if(transferingBallToIndexer == true)
                                {
                                    if(indexerEntranceSwitchState == BALL_PRESENT)
                                    {
                                        indexerStop();
                                        transferingBallToIndexer = false;
                                        //System.out.println("INDEXER OFF");
                                        ballCount++;
                                        if(ballCount >= MAX_NUM_BALLS)  //checks if 5 balls in indexer
                                        {
                                            //System.out.println("5 balls");
                                            reachedMaxCapacity = true;
                                        }
                                    }
                                }
                            }
                        }           
                    } 
                }
                Timer.delay(0.01);
            }
        });      
        indexerThread.start(); //repeats thread
    }   // end of startIndexerThread()
    
    public void stopIndexerThread()
    {
        indexerThread.interrupt();
    }

    public void resetData()
    {
        ballCount = 0;
    }

    public void setManualOverride(boolean overrideState)
    {
        reverseIndexerState = overrideState;
    }

    public void setShooterIsRunning(boolean isRunning)
    {

        this.shooterRunning = isRunning;
    }

    public void setShooterRamping(boolean isRamping)
    {
        shooterRamping = isRamping;
    }

    public int getBallCount()
    {
        return ballCount;
    }
    public boolean reachedMaxCapacity()
    {
        return reachedMaxCapacity;
    }

    public double getUltraSonicSensorReading()
    {
        return ballSensor.getRangeInches();     //Ultrasonic range in inches
    }

    public boolean isBallInIntexer()        //returns if ball is sensed in range by ultrasonic
    {
        sensorRange = ballSensor.getRangeInches();

        if (sensorRange < BALL_IN_RANGE_THRESHOLD)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void indexerStart()
    {
        if(testMode == false)
        {
            shooterRunning = true;
            //indexerMtrCtrl.set(BELT_SPEED_SHOOT);
        }
    }
    public void indexerStop()
    {
        if(testMode == false)
        {
            shooterRunning = false;
            indexerMtrCtrl.set(0.0);
        }
    }
    public void indexerReversedOn()
    {   
        //indexerMtrCtrl.set(-BELT_SPEED_LOAD);
        reverseIndexerState = true;
    }

    public void indexerReversedOff()
    {
        reverseIndexerState = false;
        prevReverseIndexerState = false;
    }

    public void clearSwitchState()
    { 
        indexerEntranceSwitchState = BALL_PRESENT; 
        indexerExitSwitchState     = BALL_PRESENT;
        
        transferingBallToIndexer = false;
    }

    public void debugSmartDashboard()
    {
        SmartDashboard.putNumber("Ball Range", sensorRange);
        SmartDashboard.putBoolean("Transfering", transferingBallToIndexer);
        SmartDashboard.putBoolean("SW-Entrance", indexerEntranceSwitch.get());
        SmartDashboard.putBoolean("SW-Exit", indexerExitSwitch.get());
        SmartDashboard.putNumber("Ultrasonic", getUltraSonicSensorReading());
        SmartDashboard.putBoolean("Shooter running", shooterRunning);
    }
    public void smartDashboard()
    {
        SmartDashboard.putNumber("Ball Count", ballCount);
        SmartDashboard.putBoolean("Reached max capacity", reachedMaxCapacity);
    }
}