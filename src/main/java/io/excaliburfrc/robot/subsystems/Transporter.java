package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ejml.data.ElementLocation;

import static frc.robot.Constants.TranporterConstants.*;


public class Transporter extends SubsystemBase {
  private final CANSparkMax lower = new CANSparkMax(LOWER_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax upper = new CANSparkMax(UPPER_MOTOR_ID, MotorType.kBrushless);
  private final ColorSensorV3 color = new ColorSensorV3(I2C.Port.kOnboard); // FIXME
  private final Ultrasonic sonic = new Ultrasonic(PING, ECHO);


  public enum Mode {
    // (lower, upper)
    SHOOT(0.4, 0.4),
    IN(0.3, 0),
    OUT(-0.4, 0),
    OFF(0, 0),
    CHECK(0.3, 0);

    Mode(double lr, double ur) {
      lower = lr;
      upper = ur;
    }

    public final double lower, upper;
  }


//  public void activate(boolean skip){
//    if (isColorReady() || skip) { // Check if there is a ball in the lower engine
//
//      if (color.getColor().equals(getAlliance()) || skip) { // FIXME
//        if (isSonicReady()) { // Checks of there is a ball in the upper engine
//          stop(); // Stops both engines
//        } else if (skip){
//          activate(true);
//        } else {
//          lower.set(Mode.IN.lower); // pushes the ball to the top engine
//          upper.set(Mode.IN.upper);// catches the ball
//          activate(true);
//        }
//      } else {
//        lower.set(Mode.OUT.lower);
//      }
//    } else {
//      lower.set(Mode.CHECK.lower);
//    }
//  }

  public DriverStation.Alliance getAlliance() {
    return DriverStation.getAlliance();
  }

  public boolean isSonicReady() {
    return sonic.getRangeMM() > SONIC_LIMIT;
  }

  public boolean isColorReady() {
    return color.getProximity() > COLOR_LIMIT;
  }

  public void stopLower() {
    lower.set(Mode.OFF.lower);
  }

  public void stopUpper() {
    upper.set(Mode.OFF.lower);
  }

  public void stop() {
    stopLower();
    stopUpper();
  }

  @Override
  public void periodic() {
    if (isColorReady()) { // Check if there is a ball in the lower engine

      if (color.getColor().equals(getAlliance())) { // FIXME
        if (isSonicReady()) { // Checks of there is a ball in the upper engine
          stop(); // Stops both engines
        } else {
          lower.set(Mode.IN.lower); // pushes the ball to the top engine
          upper.set(Mode.IN.upper);// catches the ball
        }
      } else {
        lower.set(Mode.OUT.lower);
      }
    } else {
      lower.set(Mode.CHECK.lower);
    }
  }
}
