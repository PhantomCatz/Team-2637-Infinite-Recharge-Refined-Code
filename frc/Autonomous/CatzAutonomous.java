package frc.Autonomous;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.Mechanisms.*;
import frc.robot.*;

public class CatzAutonomous
{

    public static boolean checkBoxL;
    public static boolean checkBoxM;
    public static boolean checkBoxR;

    public boolean prev_boxL = false;
	public boolean prev_boxM = false;
	public boolean prev_boxR = false;

    public final double STOP_THRESHOLD_ANGLE = 2;
    public final double SLOW_THRESHOLD_ANGLE = 20;

    public boolean runningRadialTurn = false;
    public double angleGoal;
    public double angleToTurn;

    public double currentEncPosition;
    public double turnRateRadians;

    public double r1;
    public double r2;
    public double s1Dot;
    public double s2Dot;
    public double s1Conv;
    public double s2Conv;

    public Timer turnT;
    public Timer driveT1;
    public Timer driveTWait;
    public Timer shooterTimer;

    public final double WAIT_TIME_THRESHOLD = 5.0;
    public double totalTime = 0;

    public double lVelocity;
    public double rVelocity;

    public double leftInitialEncoderPos;
    public double rightInitialEncoderPos;

    public boolean runningDistanceDrive = false;
    public boolean driveBackwards = false;
    public boolean monitoringTime = false;

    public double distanceGoal;
    public double distanceMoved;
    public final double STOP_THRESHOLD_DIST = 1;
    public final double SLOW_THRESHOLD_DIST = 30;

    public final double ENCODER_COUNTS_PER_INCH_LT = 1014.5; //without weight: 1046.6
    public final double ENCODER_COUNTS_PER_INCH_RT = 964;    //without weight: 1025.7

    public final double TO_RADIANS = Math.PI/180;
    public final double TO_DEGREES = 180/Math.PI;

    public CatzAutonomous()
    {
        turnT        = new Timer();
        driveT1      = new Timer();
        driveTWait   = new Timer();
        shooterTimer = new Timer();
    }

    public void resetTotalTime()
    {
        totalTime = 0;
    }

    public boolean simulateShoot()
    {
        boolean status = false;

        if(shooterTimer.get() > 3)
        {
            status = true;
        }
        return status;
    }

    public void monitorTimer()
    {
        if(monitoringTime)
        {    
            if(driveTWait.get() > WAIT_TIME_THRESHOLD)
            {
                setDistanceGoal(103.5, 16000);
            }
        }
    }

    public boolean monitorEncoderPosition()
    {
        boolean status = false;

        if (runningDistanceDrive == true)
        {
            currentEncPosition = Robot.driveTrain.getIntegratedEncPosition("LT");
            distanceMoved = Math.abs(leftEncoderDistanceMoved(currentEncPosition));
           
            SmartDashboard.putNumber("Encoder Position", currentEncPosition);
            SmartDashboard.putNumber("Initial Encoder Distance", leftInitialEncoderPos);
            SmartDashboard.putNumber("Distance Moved", distanceMoved);
            SmartDashboard.putNumber("Delta Encoder", (currentEncPosition - leftInitialEncoderPos));
            SmartDashboard.putNumber("drive timer", driveT1.get());

            //System.out.println((currentEncPosition - leftInitialEncoderPos) + " = " + currentEncPosition + " - " + leftInitialEncoderPos);
            double distanceToGoal;
            if(distanceGoal > 0)
            {
                distanceToGoal = distanceGoal - distanceMoved;
            }
            else
            {
                distanceToGoal = distanceGoal + distanceMoved;
            }
                

            SmartDashboard.putNumber("Distance To Goal", distanceToGoal);

            SmartDashboard.putNumber("total time", totalTime);
            if (distanceToGoal < STOP_THRESHOLD_DIST)
            {
                Robot.driveTrain.setTargetVelocity(0);
                runningDistanceDrive = false;
                status = true;
            }
            else if (distanceToGoal < SLOW_THRESHOLD_DIST && distanceToGoal < distanceGoal*0.5)
            {
                if(Robot.driveTrain.getIntegratedEncVelocity("LT") > 3000)
                    Robot.driveTrain.setTargetVelocity(Robot.driveTrain.getIntegratedEncVelocity("LT")*0.98);
                
            }

        }

        return status;
    }

    public void setDistanceGoal(double inches, int speed)
    {
        if(!runningDistanceDrive)
        {   
            driveT1.reset();
            monitoringTime = false;
            runningDistanceDrive = true;
            leftInitialEncoderPos = Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(0);
            if (inches < 0)
            {
                driveT1.start();
                distanceGoal = -inches;
                Robot.driveTrain.setTargetVelocity(-speed);
            }
            else    
            {
                driveT1.start();
                distanceGoal = inches;
                Robot.driveTrain.setTargetVelocity(speed);
            }
        }
    }

 /*   public boolean monitorAngle()
    {
        boolean status = false;
        if (runningRadialTurn == true)
        {
            double currentAngle = Math.abs(Robot.navx.getAngle());
            SmartDashboard.putNumber("Current Angle", currentAngle);
            SmartDashboard.putNumber("turn Time", turnT.get());

            angleToTurn = angleGoal - currentAngle;

            SmartDashboard.putNumber("Angle to Turn", angleToTurn);
            SmartDashboard.putNumber("Angle to Turn Plot", angleToTurn);
            SmartDashboard.putNumber("Angle Goal", angleGoal);

            SmartDashboard.putNumber("left enc velocity", Robot.driveTrain.getIntegratedEncVelocity("LT"));

            if(angleToTurn < STOP_THRESHOLD_ANGLE)
            {
                Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, 0);
                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, 0);  
                turnT.stop();
                status = true;
                runningRadialTurn = false;
            }
           else if (angleToTurn < SLOW_THRESHOLD_ANGLE && angleToTurn < (angleGoal * 0.5))
            {

                lVelocity = Robot.driveTrain.getIntegratedEncVelocity("LT");
                rVelocity = Robot.driveTrain.getIntegratedEncVelocity("RT");

                if(lVelocity > 5000 && rVelocity > 5000)
                {
                    Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, Robot.driveTrain.getIntegratedEncVelocity("LT")*0.99);
                    Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, Robot.driveTrain.getIntegratedEncVelocity("RT")*0.99);
                }

            }

        }

        return status;

    }
    
    /**
     * @param angle - The target angle to turn; A negative angle results in a left turn
     * @param speed - The base speed to apply to the drive train; Measured in counts/100ms
     * @param ratio - The ratio between the two sides of the drive train; Setting 0.5 will result in one side moving at half the speed of the other
     */
 /*   
    public void setAngleGoal(double angle, int speed, double ratio)
    {
        if(!runningRadialTurn)
        {


            angleGoal = angle;
            runningRadialTurn = true;
            Robot.navx.reset();
            turnT.start();

            if(angle < 0)
            {

                Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, speed * ratio);
                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, -speed);

            }
            else
            {

                Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, speed);
                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, -speed * ratio);

            }


            
        }
    }

    public void radialTurn(double radiusOfCurvature, double turnRateDegrees, double targetAngleDegrees)
    {
        turnT.reset();

        r1 = radiusOfCurvature;
        r2 = radiusOfCurvature + (7.0/3.0);

        turnRateRadians = turnRateDegrees*TO_RADIANS;

        s1Dot = r1 * turnRateRadians;
        s2Dot = r2 * turnRateRadians;

        s1Conv = s1Dot * ENCODER_COUNTS_PER_INCH_RT * 12 *(1.0/10.0);
        s2Conv = s2Dot * ENCODER_COUNTS_PER_INCH_LT * 12 *(1.0/10.0);

        SmartDashboard.putNumber("s1", s1Dot);
        SmartDashboard.putNumber("s2", s2Dot);
        SmartDashboard.putNumber("s1Conv", s1Conv);
        SmartDashboard.putNumber("s2Conv", s2Conv);
        SmartDashboard.putNumber("Target Angle", targetAngleDegrees);
        SmartDashboard.putNumber("Turn Rate Degrees", turnRateDegrees);

        double targetAngleRadians = targetAngleDegrees * TO_RADIANS;
        double timeOut = (targetAngleRadians)/turnRateRadians;
       
        turnT.start();

        SmartDashboard.putNumber("Time Out", timeOut);

        double deltaTime;
        double timeStart = turnT.get();

        while ((turnT.get() - timeStart) < 2)
        {
            Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, s2Conv);
            Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, -s1Conv * DRIVE_STRAIGHT_PID_TUNING_CONSTANT);

            Robot.driveTrain.drvTrainMtrCtrlLTBack.follow(Robot.driveTrain.drvTrainMtrCtrlLTFrnt);
            Robot.driveTrain.drvTrainMtrCtrlRTBack.follow(Robot.driveTrain.drvTrainMtrCtrlRTFrnt);

            deltaTime = turnT.get() - timeStart;
            SmartDashboard.putNumber("Delta time", deltaTime);
        }
        if(turnT.get() > timeOut)
        {
            Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, 0);
            Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, 0);
        }

    }
    */

    public double leftEncoderDistanceMoved(double encoderPosition)
    {  
        //System.out.println((getIntegratedEncPosition("LT") - leftInitialEncoderPos) + " = " + getIntegratedEncPosition("LT") + " - " + leftInitialEncoderPos);
        return (encoderPosition - leftInitialEncoderPos) / ENCODER_COUNTS_PER_INCH_LT;
    }

    public double rightEncoderDistanceMoved()
    {   
        rightInitialEncoderPos = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);
        return (Robot.driveTrain.getIntegratedEncPosition("RT") - rightInitialEncoderPos) * ENCODER_COUNTS_PER_INCH_RT;
    }

}