package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase{
    
    private static TankDrive m_instance;
    private CANSparkMax m_motorLT;
    private CANSparkMax m_motorLB;
    private CANSparkMax m_motorRT;
    private CANSparkMax m_motorRB;

    public TankDrive(){
       m_motorLB = new CANSparkMax(0, MotorType.kBrushless);
       m_motorRB = new CANSparkMax(0, MotorType.kBrushless);
       m_motorRT = new CANSparkMax(0, MotorType.kBrushless);
       m_motorRT = new CANSparkMax(0, MotorType.kBrushless);
       m_motorLB.follow(m_motorLT);
       m_motorRB.follow(m_motorRT);
    }

    public static TankDrive getInstance(){
        if(m_instance==null)
            m_instance = new TankDrive();

        return m_instance;
    }

    public void drive(double speedL, double speedR){  
        m_motorLT.set(speedL);
        m_motorRT.set(speedR);
    }

    public void stop(){
        drive(0, 0);
    }


}
