package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Utils.Vector2d;

public class TankDrive extends SubsystemBase{
    
    private static TankDrive m_instance;
    private CANSparkMax m_motorLT;
    private CANSparkMax m_motorLB;
    private CANSparkMax m_motorRT;
    private CANSparkMax m_motorRB;
    private AHRS m_gyro;
    private Vector2d m_currentPoint;
    private double m_lastTimeMilis;
    private final double m_gearRatio=1, m_diameter=1;
    
    public TankDrive(){
       m_motorLB = new CANSparkMax(0, MotorType.kBrushless);
       m_motorRB = new CANSparkMax(0, MotorType.kBrushless);
       m_motorRT = new CANSparkMax(0, MotorType.kBrushless);
       m_motorRT = new CANSparkMax(0, MotorType.kBrushless);
       m_motorLB.follow(m_motorLT);
       m_motorRB.follow(m_motorRT);
       m_lastTimeMilis = System.currentTimeMillis();
       m_gyro = new AHRS(Port.kMXP);
       m_motorLB.getEncoder().setVelocityConversionFactor((360 / m_gearRatio)*m_diameter);
    }

    public static TankDrive getInstance(){
        if(m_instance==null)
            m_instance = new TankDrive();

        return m_instance;
    }
    
    
    @Override
    public void periodic() {
        double currentTimeMilis = System.currentTimeMillis();
        double deltaTimeSec = (currentTimeMilis - m_lastTimeMilis) / 1000.0;
        double leftCurrentSpeed = m_motorLT.getEncoder().getVelocity();
        double rightCurrentSpeed = m_motorRT.getEncoder().getVelocity();
        Vector2d velocityVector2d = new Vector2d(0,leftCurrentSpeed+rightCurrentSpeed);
        velocityVector2d.rotate(Math.toRadians(m_gyro.getAngle()));
        m_lastTimeMilis = currentTimeMilis;
        m_currentPoint.add(velocityVector2d.mul(deltaTimeSec));
    }

    public void drive(double speedL, double speedR){  
        m_motorLT.set(speedL);
        m_motorRT.set(speedR);
    }

    public void driveStrightMpS(double speedMpS){
        m_motorLT.getPIDController().setReference(speedMpS, ControlType.kVelocity);
        m_motorRT.getPIDController().setReference(speedMpS, ControlType.kVelocity);
    }

    public void stop(){
        drive(0, 0);
    }


}
