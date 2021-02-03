/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String gyroAuto = "NavX auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  Timer clock = new Timer();
  WPI_TalonSRX left = new WPI_TalonSRX(4);
  WPI_TalonSRX Left = new WPI_TalonSRX(3);
  WPI_TalonSRX Right = new WPI_TalonSRX(1);
  WPI_TalonSRX right = new WPI_TalonSRX(0);
  WPI_TalonSRX intakewheels = new WPI_TalonSRX(5);
  WPI_TalonSRX moveshooter = new WPI_TalonSRX(2);
  WPI_TalonSRX climber = new WPI_TalonSRX(6);
  // WPI_TalonSRX colors = new WPI_TalonSRX(7);
  DifferentialDrive arcade = new DifferentialDrive(Left, Right);
  Joystick joy = new Joystick(0);
  CANSparkMax shooter = new CANSparkMax(2, MotorType.kBrushless);
  Compressor compressor = new Compressor(0);

  Solenoid kicker = new Solenoid(6);
  //Solenoid lights = new Solenoid(7);
  Solenoid intakeSolenoidIn = new Solenoid(4);
  Solenoid intakeSolenoidOut = new Solenoid(5);
  Solenoid climberSolenoid = new Solenoid(2);

  //CameraServer camera = CameraServer.getInstance();  
  
  DigitalInput tHood = new DigitalInput(1);
  DigitalInput bHood = new DigitalInput(0);

  boolean on = true;
  int stop = 0;
  int flag = 0;
  int shoot = 0;
  int a = -1;
  int x = 0;
  int g = 0;
  int red = 0;
  double turn = 0;
  double forward = 0.75;
  double speed = 0;
  double time = 0.5;
  String name = "robot is working";
  double shootSpeed = 1;

  // X52 button mapping 
  double axisX;// axis 1
  double axisY;// axis 2
  double axisZ;// axis 3
  double rotX; // axis 4
  double rotY; // axis 5
  double rotZ; // axis 6
  double slider;// axis 7
  int trigger = 1;
  int Fire = 2;
  int A = 3;
  int B = 4;
  int C = 5;
  int pink = 6;
  int D = 7;
  int E = 8;
  int T1 = 9;
  int T2 = 10;
  int T3 = 11;
  int T4 = 12;
  int T5 = 13;
  int T6 = 14;
  int povUp = 16;
  int povRight = 17;
  int povDown = 18;
  int povLeft = 19;
  int thumbUp = 20;
  int thumbRight = 21;
  int thumbDown = 22;
  int thumbLeft = 23;
  int ModeG = 24;
  int ModeO = 25;
  int ModeR = 26;
  int i = 30;
  int button = 31;
  int scroll = 32;

  // I2C.Port coloPort = I2C.Port.kOnboard;
   I2C.Port navPort = I2C.Port.kMXP;
  //  DigitalInput limit = new DigitalInput(0); 
  // ColorSensorV3 color = new ColorSensorV3(coloPort);
   AHRS navX = new AHRS(navPort);
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("NavX auto",gyroAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    compressor.setClosedLoopControl(true);
    //camera.startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    left.follow(Left);
    right.follow(Right);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    clock.start();
  }
  //initial end point, ramp up distance, final speed, input varible, total distance
  public double Speed (double E, double D, double S,double Var, double Dis ){
    if (Var<E){
      return speed = ((E - Var)/D)*S;
    }else if(Var>Dis-D) {
      return speed = ((Dis - Var)/D)*S;
    }else{
      return speed = S;
    }
  }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case gyroAuto:
      if(navX.getYaw() < 29.14){
        arcade.arcadeDrive(0, Speed(0.5,0.5,0.5,navX.getYaw(),29.14));
      }else{
        if(clock.get() < 1.1){
          shooter.set(1);
          if(clock.get() < time){
            kicker.set(true);
            shoot += 1;
            time += 0.25;
          }else{
            kicker.set(false);
          }
        }else{
          shooter.set(0);
          kicker.set(false);
        }
        if(shoot == 3){
          if(navX.getYaw() < 180){
            arcade.arcadeDrive(0, Speed(0.5,0.5,0.5,navX.getYaw(),180));
            intakeSolenoidOut.set(on);
          }else{
            shoot = 4;
          } 
        }else if(shoot == 4){
          if(navX.getDisplacementX() < 6.57){
            arcade.arcadeDrive(Speed(0.5,0.5,1,navX.getDisplacementX(),6.57), 0);
            intakeSolenoidOut.set(false);
            intakewheels.set(1);
          }else{
            shoot = 5;
          }
        }else if(shoot == 5){
          if(navX.getYaw() < 360){
            arcade.arcadeDrive(0, Speed(0.5,0.5,0.5,navX.getYaw(),360));          
          }else{
            shoot = 6;
          }
        }else if(shoot == 6){
          if(navX.getDisplacementX() < 13.14){
            arcade.arcadeDrive(Speed(0.5,0.5,1,navX.getDisplacementX(),13.14), 0);
          }else{
            shoot = 7;
          }
        }else if(shoot == 7){
          if(navX.getYaw() < 29.14){
            arcade.arcadeDrive(0, Speed(0.5,0.5,0.5,navX.getYaw(),29.14));
          }else{
            clock.reset();
            shoot = 8;
          }
        }else if (shoot == 8){
          if(clock.get() < 1.1){
            shooter.set(1);
            if(clock.get() < time){
              kicker.set(true);
              shoot += 1;
              time += 0.25;
            }
          }else{
            shooter.set(0);
            kicker.set(false);
          }
        }
      }
        break;
      case kCustomAuto:
        arcade.arcadeDrive(speed, turn);
        if (clock.get()<5){
          speed = clock.get()/5;
        }else if(clock.get()<5 && clock.get()<5.5){
          speed = 1;
        }else if(clock.get()<5.5 && clock.get()<10.5){
          speed = (10.5-clock.get())/5;
        }
        if(clock.get()<10.5 && clock.get()<12){
          turn = ((12-clock.get())/1.5)*0.75;
        }else if (clock.get()<12 && clock.get()<13.5){
          turn = 0.75;
        }else if (clock.get()<13.5 && clock.get()<15){
          turn = ((15-clock.get())/1.5)*0.75;
        }
        if (clock.get()<15 && clock.get()<16.5){
          speed = (16.5-clock.get())/1.5;
        }else if(clock.get()<16.5 && clock.get()<18){
          speed = 1;
        }else if(clock.get()<18 && clock.get()<19.5){
          speed = (19.5-clock.get())/1.5;
        }
        if(clock.get()<19.5 && clock.get()<21){
          turn = ((21-clock.get())/1.5)*0.75;
        }else if (clock.get()<21 && clock.get()<22.5){
          turn = 0.75;
        }else if (clock.get()<22.5 && clock.get()<24){
          turn = ((24-clock.get())/1.5)*0.75;
        }
        if(clock.get()<24 && clock.get()<26.5){
          speed = ((26.5-clock.get())/1.5)*0.25;
        }else if (clock.get()<26.5 && clock.get()<28){
          speed = 0.25;
        }else if (clock.get()<28 && clock.get()<29.5){
          speed = ((29.5 - clock.get())/1.5)*0.25;
        }else{
          speed = 0;
          turn = 0;
        }
      

      break;
      case kDefaultAuto:
      default:
        if(navX.getDisplacementX() < 1){
          arcade.arcadeDrive(-1, 0);
        }else{
          arcade.arcadeDrive(0, 0);
        }
        break;
      }
    }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    arcade.arcadeDrive(-1*joy.getY(), joy.getRawAxis(5));
    shooterMech();
    intakeMech();
    //climberMech();
  }
  public void shooterMech(){
    if(joy.getRawButton(Fire)){
      shooter.set(shootSpeed);
    }else{
      shooter.set(stop);
    }
    if(joy.getRawButton(T6)){
      shootSpeed = 1;
      SmartDashboard.putBoolean("Shooter Speed", true);
    }else if(joy.getRawButton(T5)){
      shootSpeed = 0.5;
      SmartDashboard.putBoolean("Shooter Speed", false);
    }

    if(joy.getRawButton(D)){
      kicker.set(on);
    }else{
      kicker.set(false);
    }
    if(joy.getRawButton(thumbUp)){
      moveshooter.set(0.25);
    }else if(joy.getRawButton(thumbDown)){
      moveshooter.set(-0.25);
    }else{
      moveshooter.set(0);
    }

    SmartDashboard.putBoolean("Hood Top Limit", tHood.get());
    SmartDashboard.putBoolean("Hood Bottom Limit", bHood.get());
  }
  public void intakeMech(){
    if(joy.getRawButton(B)){
      intakeSolenoidIn.set(on);
      intakeSolenoidOut.set(false);
    }else if(joy.getRawButton(A)){
      intakeSolenoidIn.set(false);
      intakeSolenoidOut.set(on);
    }else{
      intakeSolenoidIn.set(false);
      intakeSolenoidOut.set(false);
    }
    if(joy.getRawButton(trigger)){
      intakewheels.set(1);
    }else if(joy.getRawButton(pink)){
      intakewheels.set(-1);
    }else{
      intakewheels.set(0);
    }
  }
  public void climberMech(){
    if(joy.getRawButton(povDown)){
      climberSolenoid.set(on);
    }else{
      climberSolenoid.set(false);
    }
    if(joy.getRawButton(T3)){
      climber.set(1);
    }else{
      climber.set(0);
    }
    
    
  }
/*
int Cflag = 0;
int CoFlag = 0;
char FMS;
public void colorWheelMech(){
if(DriverStation.getInstance().getGameSpecificMessage().isBlank()){
    if(joy.getRawButton(povUp) && Cflag == 0){
      Cflag = 1;
    }
    else if(!joy.getRawButton(povUp) && Cflag == 1){
      Cflag = 2;
    }
    else if(joy.getRawButton(povUp) && Cflag == 2){
      Cflag = 3;
    }
    else if(!joy.getRawButton(povUp) && Cflag == 3){
      Cflag = 4;
    }
    red = color.getRed();
    if(Cflag == 1 || Cflag == 2){
      if(x<7){
        colors.set(1); 
      }else {
        colors.set(0);
      }
      if(red>2000 && g == 0){
        x = x + 1;
        g = 1;
      }else if (red < 2000 && g == 1){
        g=0;
      }
    }else{
      colors.set(0);
    }
  }else{
    FMS = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
    if(joy.getRawButton(povUp) && CoFlag == 0){
      CoFlag = 1;
    }
    else if(!joy.getRawButton(povUp) && CoFlag == 1){
      CoFlag = 2;
    }
    else if(joy.getRawButton(povUp) && CoFlag == 2){
      CoFlag = 3;
    }
    else if(!joy.getRawButton(povUp) && CoFlag == 3){
      CoFlag = 4;
    }
    
    if(CoFlag == 1 || CoFlag == 2){
      switch(FMS){
        case 'Y':
        if(color.getRed()>800){
          colors.set(0);
        }else{
          colors.set(1);
        }
        break;
        case 'B':
          if(color.getGreen()>2000 && color.getRed()>1000){
            colors.set(0);
          }else{
            colors.set(1);
          }
        break;
        case 'R':
        if(color.getGreen()>1000){
          colors.set(0);
        }else{
          colors.set(1);
        }
        break;
        case 'G':
        if(color.getBlue()>1500){
          colors.set(0);
        }else{
          colors.set(1);
        }
        break;
      }
    }else{
      colors.set(0);
    }
  }
  System.out.println("wrong color red " + color.getRed());
  SmartDashboard.putNumber("wrong color red", red);
}*/

int OO = 0;
int pixyX;
int center = 150;
public void Pixy(){
  if(joy.getRawButton(button) && OO == 0){
    OO = 1;
  }else if(!joy.getRawButton(button) && OO == 1){
    OO = 2;
  }else if(joy.getRawButton(button) && OO == 2){
    OO = 3;
  }else if(!joy.getRawButton(button) && OO == 3){
    OO = 4;
  }
  if(OO == 1 || OO == 2){
    if(pixyX < center){
      arcade.arcadeDrive(0, Speed(pixyX + 10, 10, 0.3, pixyX, center - pixyX));
    }else if(pixyX > center){
      arcade.arcadeDrive(0, Speed(pixyX - 10, 10,0.3, pixyX, pixyX - center));
    }else{
      arcade.arcadeDrive(0, 0);
    }
  }else{
    arcade.arcadeDrive(0,0);
  }
}

public void navX() {
  SmartDashboard.putNumber("gyro X", navX.getRawGyroX());
  SmartDashboard.putNumber("gyro Y", navX.getRawGyroY());
  SmartDashboard.putNumber("gyro Z", navX.getRawGyroZ());
  SmartDashboard.putNumber("accel X", navX.getRawAccelX());
  SmartDashboard.putNumber("accel Y", navX.getRawAccelY());
  SmartDashboard.putNumber("accel Z", navX.getRawAccelZ());
  SmartDashboard.putNumber("velocity X", navX.getVelocityX());
  SmartDashboard.putNumber("velocity Y", navX.getVelocityY());
  SmartDashboard.putNumber("velocity Z", navX.getVelocityZ());
}
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
