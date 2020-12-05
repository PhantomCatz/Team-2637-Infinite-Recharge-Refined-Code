package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.DigitalInput;  //Currently using SparkMax Data Port
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CatzIntake 
{
    private final int INTAKE_FIGURE_8_MC_CAN_ID             = 10;
    private final int INTAKE_ROLLER_MC_CAN_ID               = 11;
    private final int INTAKE_DEPLOY_MC_CAN_ID               = 12;

    private final double MTR_POWER_FIGURE8                  = -0.9;
    private final double MTR_POWER_ROLLER                   =  0.6;

    private final double COMPRESSION_POWER                  = 0.3;

    private final double INTAKE_MOTOR_POWER_START_DEPLOY    =  0.25;
    private final double INTAKE_MOTOR_POWER_END_DEPLOY      =  0.0;
    private final double INTAKE_MOTOR_POWER_START_STOW      = -0.25;
    private final double INTAKE_MOTOR_POWER_END_STOW        = -0.2;

    final double INTAKE_THREAD_WAITING_TIME                 = 0.050;
    final double DEPLOY_REDUCE_POWER_TIME_OUT_SEC           = 0.400;
    final double STOW_REDUCE_POWER_TIME_OUT_SEC             = 0.350;

    private final int INTAKE_MODE_NULL                      = 0;
    private final int INTAKE_MODE_DEPLOY_START              = 1;
    private final int INTAKE_MODE_DEPLOY_REDUCE_POWER       = 2;
    private final int INTAKE_MODE_STOW_START                = 3;
    private final int INTAKE_MODE_STOW_REDUCE_POWER         = 4;
    private final int INTAKE_MODE_MANUAL_CONTROL            = 5;

    private WPI_VictorSPX intakeFigure8MtrCtrl;
    public  WPI_TalonSRX  intakeRollerMtrCtrl; // changed to public because dataport is being used on drivetrain?

    private CANSparkMax intakeDeployMtrCtrl;

    private CANDigitalInput intakeDeployedLimitSwitch;
    private CANDigitalInput intakeStowedLimitSwitch;

    private int deployPowerCountLimit = 0;
    private int stowPowerCountLimit   = 0;
    private int timeCounter = 0;

    private Thread intakeThread;

    private int intakeMode = INTAKE_MODE_NULL;

    boolean intakeDeployed = false;
    
    public CatzIntake() 
    {
        intakeFigure8MtrCtrl = new WPI_VictorSPX(INTAKE_FIGURE_8_MC_CAN_ID); 
        intakeRollerMtrCtrl  = new WPI_TalonSRX(INTAKE_ROLLER_MC_CAN_ID);
        intakeDeployMtrCtrl  = new CANSparkMax (INTAKE_DEPLOY_MC_CAN_ID, MotorType.kBrushless);

        intakeDeployedLimitSwitch = intakeDeployMtrCtrl.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        intakeStowedLimitSwitch   = intakeDeployMtrCtrl.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        //Reset configuration
        intakeFigure8MtrCtrl.configFactoryDefault();
        intakeRollerMtrCtrl.configFactoryDefault();

        intakeDeployMtrCtrl.restoreFactoryDefaults();

        //Set roller MC to coast intakeMode
        intakeFigure8MtrCtrl.setNeutralMode(NeutralMode.Coast);
        intakeRollerMtrCtrl.setNeutralMode(NeutralMode.Coast);

        //Set deploy MC to brake intakeMode
        intakeDeployMtrCtrl.setIdleMode(IdleMode.kBrake);

        deployPowerCountLimit = (int) Math.round((DEPLOY_REDUCE_POWER_TIME_OUT_SEC / INTAKE_THREAD_WAITING_TIME) + 0.5);
        stowPowerCountLimit   = (int) Math.round((STOW_REDUCE_POWER_TIME_OUT_SEC   / INTAKE_THREAD_WAITING_TIME) + 0.5);
        SmartDashboard.putNumber("stow count limit", stowPowerCountLimit);
        SmartDashboard.putNumber("deploy count limit", deployPowerCountLimit);
    }

    // ---------------------------------------------ROLLER---------------------------------------------
    public void intakeRollerIn()
    {
        intakeFigure8MtrCtrl.set(ControlMode.PercentOutput, -MTR_POWER_FIGURE8);
        intakeRollerMtrCtrl.set(ControlMode.PercentOutput, -MTR_POWER_ROLLER);
    }
    public void intakeRollerOut()
    {
        intakeFigure8MtrCtrl.set(ControlMode.PercentOutput, MTR_POWER_FIGURE8);
        intakeRollerMtrCtrl.set(ControlMode.PercentOutput, MTR_POWER_ROLLER);
    }
    public void intakeRollerOff()
    {
        intakeFigure8MtrCtrl.set(ControlMode.PercentOutput, 0.0);
        intakeRollerMtrCtrl.set(ControlMode.PercentOutput,  0.0);
    }

    // ---------------------------------------------DEPLOY/STOW---------------------------------------------
    public void intakeControl()
    {
        intakeThread = new Thread(() ->
        { 
            double shootTime;

            while(true)
            {
                shootTime = Robot.dataCollectionTimer.get();
                switch(intakeMode)
                {
                    case INTAKE_MODE_DEPLOY_START:
                        intakeDeployMtrCtrl.set(INTAKE_MOTOR_POWER_START_DEPLOY);
                        intakeMode = INTAKE_MODE_DEPLOY_REDUCE_POWER;

                        timeCounter++;
                    break;

                    case INTAKE_MODE_DEPLOY_REDUCE_POWER:
                        if(timeCounter > deployPowerCountLimit)
                        {
                            intakeDeployMtrCtrl.set(INTAKE_MOTOR_POWER_END_DEPLOY);
                            intakeMode = INTAKE_MODE_NULL;
                            intakeDeployed = true;
                        }
                        timeCounter++;
                    break;

                    case INTAKE_MODE_STOW_START:
                        intakeDeployMtrCtrl.set(INTAKE_MOTOR_POWER_START_STOW);
                        intakeMode = INTAKE_MODE_STOW_REDUCE_POWER;

                        timeCounter++;
                    break;

                    case INTAKE_MODE_STOW_REDUCE_POWER:
                        if(timeCounter > stowPowerCountLimit)
                        {
                            intakeDeployMtrCtrl.set(INTAKE_MOTOR_POWER_END_STOW);
                            //intakeMode = INTAKE_MODE_NULL;
                            intakeDeployed = false;
                        }
                        timeCounter++;
                    break;

                    case INTAKE_MODE_MANUAL_CONTROL:
                        if(Robot.xboxAux.getBumper(Hand.kLeft)){
                            intakeDeployMtrCtrl.set(INTAKE_MOTOR_POWER_START_DEPLOY);
                        }else if(Robot.xboxAux.getBumper(Hand.kRight)){
                            intakeDeployMtrCtrl.set(INTAKE_MOTOR_POWER_START_STOW);
                        }else{
                            intakeDeployMtrCtrl.set(0.0);
                        }
                    break;

                    default:
                        intakeDeployMtrCtrl.set(0.0);
                    break;
                }
                Timer.delay(INTAKE_THREAD_WAITING_TIME); //put at end
            }   
        });
        intakeThread.start();
    }
    public void changeMode(int mode){
        intakeMode = mode;
    }
    public int getMode(){
        return intakeMode;
    }
    public void deployIntake()
    {
        timeCounter = 0;
        intakeMode = INTAKE_MODE_DEPLOY_START;
        //intakeDeployMtrCtrl.set(0.25);
    }
    public void stowIntake()
    {
        timeCounter = 0;
        intakeMode = INTAKE_MODE_STOW_START;
        //intakeDeployMtrCtrl.set(-0.25);
    }
    public void stopDeploying()
    {
        intakeDeployMtrCtrl.set(0);
    }

    public void applyBallCompression()
    {
        intakeDeployMtrCtrl.set(COMPRESSION_POWER);
        /*if(intakeDeployed == true)
        {
            intakeDeployMtrCtrl.set(COMPRESSION_POWER);
        }*/
    }

    public double getDeployMotorPower()
    {
        return intakeDeployMtrCtrl.get();
    }

    // ---------------------------------------------Intake Limit Switches---------------------------------------------   
    public boolean getDeployedLimitSwitchState()
    {
        return intakeDeployedLimitSwitch.get();
    }
    public boolean getStowedLimitSwitchState()
    {
        return intakeStowedLimitSwitch.get();
    }
}