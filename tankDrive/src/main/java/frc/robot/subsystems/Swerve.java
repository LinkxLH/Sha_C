package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Utils.Vector2d;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.SwerveConsts;

public class Swerve extends SubsystemBase{

    enum SwerveMode{
        gyro_Oriented,
        robot_Oriented       
    }

    private static Swerve m_instance = new Swerve(
        new SwerveModule(0,0),
        new SwerveModule(0,0),
        new SwerveModule(0,0),
        new SwerveModule(0,0)
    );
    private SwerveModule[] m_mudolo;
    private Vector2d[] m_position = {
        SwerveConsts.RIGHT_TOP,
        SwerveConsts.LEFT_TOP,
        SwerveConsts.RIGHT_DOWN,
        SwerveConsts.LEFT_DOWN
    };
    private AHRS m_gyro = new AHRS(SPI.Port.kMXP); 


    private Swerve(SwerveModule... module){
        m_mudolo = new SwerveModule[module.length]; 
        for(int i=0; i < m_mudolo.length;i++){
            m_mudolo[i] = module[i];
        }

    }

    public static Swerve getInstance(){
        return m_instance;
    }
    
    public void driveAndRotate(Vector2d VelocityMPS, double rotationPS, SwerveMode oriention ){
        double rotationMPS = rotationPS * SwerveConsts.PARAMITER;
        for(int i = 0; i < m_mudolo.length ;i++){
            m_position[i] = m_position[i].mul(rotationMPS);
            Vector2d temp = new Vector2d(m_position[i].add(VelocityMPS));
            if(oriention == SwerveMode.gyro_Oriented){
                temp = temp.rotate(Math.toRadians(m_gyro.getAngle()));
            }
            m_mudolo[i].setState(temp);
        }
    }
    
}
