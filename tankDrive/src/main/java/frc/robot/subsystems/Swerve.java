package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.EverKit.EverGyro;
import frc.robot.Utils.EverKit.Implementations.Gyros.EverNavX;
import frc.robot.subsystems.SwerveConsts;

public class Swerve extends SubsystemBase{

    enum SwerveMode{
        gyroOriented, 
        robotOriented       
    }

    private static Swerve m_instance = new Swerve(
        new SwerveModule(0,0),
        new SwerveModule(0,0),
        new SwerveModule(0,0),
        new SwerveModule(0,0)
    );
    private SwerveModule[] m_modules;
    private Vector2d[] m_position = { // אין סיבה לשמור ככה את המיקום כבר שמרת אותו כקונסטנט למה המערך הזה נחוץ?
        SwerveConsts.RIGHT_TOP,

        SwerveConsts.LEFT_TOP,
        SwerveConsts.RIGHT_DOWN,
        SwerveConsts.LEFT_DOWN
    };
    private EverGyro m_gyro; 


    private Swerve(SwerveModule... module){
        m_modules = new SwerveModule[module.length]; 
        for(int i=0; i < m_modules.length;i++){
            m_modules[i] = module[i];
        }

        m_gyro = new EverNavX(SPI.Port.kMXP);

    }

    public static Swerve getInstance(){
        return m_instance;
    }
    
    public void driveAndRotate(Vector2d VelocityMPS, double rotationPS, SwerveMode oriention ){ // קונבנציות לא נכונות
        double rotationMPS = rotationPS /360.0 * SwerveConsts.CIRCUMFERENC;// לא כותבים ככה היקף
        Vector2d temp;
        for(int i = 0; i < m_modules.length ;i++){
            m_position[i] = m_position[i].mul(rotationMPS); // אתה ממשיך להכפיל את הוקטור סיבוב לאורך זמן משומה
            temp = m_position[i].add(VelocityMPS);
            if(oriention == SwerveMode.gyroOriented){      // מסובבים את וקטור המהירות לני החיבור 
                temp = temp.rotate(Math.toRadians(-m_gyro.getYaw()));
            }
            m_modules[i].setState(temp);
        }
    }
    
}
