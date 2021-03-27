// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private static final String Calibration = "Calibration";
  private static final String AutoNavPath = "AutoNavPath";
  private static final String AutoNavPath2 = "AutoNavPath2";
  private static final String AutoNavPath3 = "AutoNavPath3";
  private String m_autoSelected;
  WPI_TalonSRX left = new WPI_TalonSRX(4);
  WPI_TalonSRX Left = new WPI_TalonSRX(3);
  WPI_TalonSRX Right = new WPI_TalonSRX(1);
  WPI_TalonSRX right = new WPI_TalonSRX(0);
  
  MecanumDrive mecanum = new MecanumDrive(Left, left, Right, right);
  Joystick joy = new Joystick(0);
  Timer clock = new Timer();

  Solenoid intakeSolenoidIn = new Solenoid(4);
  Solenoid intakeSolenoidOut = new Solenoid(5);
  Solenoid climberSolenoid3 = new Solenoid(3);
  Solenoid climberSolenoid2 = new Solenoid(2);
  Solenoid kicker = new Solenoid(6);
  WPI_TalonSRX intakewheels = new WPI_TalonSRX(5);
  WPI_TalonSRX climber = new WPI_TalonSRX(8);
  AnalogInput tClimb = new AnalogInput(3);
  AnalogInput bClimb = new AnalogInput(2);
  CANSparkMax ShooterR = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax ShooterL = new CANSparkMax(1, MotorType.kBrushless);

  I2C.Port navPort = I2C.Port.kOnboard;
  AHRS navX = new AHRS(navPort);

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

  double shootSpeed = 1;
  boolean on = true;
  int stop = 0;
  int y = 0;
  int h = 0;
  int pos = 0;
  int x = 0;
  int z = 0;
  int pos2 = 0;
  int g = 0;
  int pos3 = 0;  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Auto choices", m_chooser);
    m_chooser.addOption("Calibration", Calibration);
    m_chooser.addOption("AutoNavPath", AutoNavPath);
    m_chooser.addOption("AutoNavPath2", AutoNavPath2);
    m_chooser.addOption("AutoNavPath3", AutoNavPath3);
    //climber.configSelectedFeedbackSensor();
    Right.setSafetyEnabled(false);
    right.setSafetyEnabled(false);
    Left.setSafetyEnabled(false);
    left.setSafetyEnabled(false);
    mecanum.setSafetyEnabled(false);

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

    //not showing accurate numbers
    SmartDashboard.putNumber("DisplacementX", navX.getDisplacementX());
    SmartDashboard.putNumber("DisplacementY", navX.getDisplacementY());
    SmartDashboard.putNumber("DisplacementZ", navX.getDisplacementZ());
  }
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    navX();
    if(joy.getRawButton(T6)){
      navX.reset();
      navX.resetDisplacement();
    }
    // SmartDashboard.putNumber("DisplaceY", navX.getDisplacementY());
    // SmartDashboard.putNumber("RotZ", navX.getYaw());

    // SmartDashboard.putNumber("RawX", navX.getRawAccelX());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    navX.reset();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    clock.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Timer.delay(0.02);
    m_autoSelected = m_chooser.getSelected();
    switch(m_autoSelected){
      case Calibration:
        //double DisCal [] = {48, 28, 48};
        double TimCal [] = {1.13, 0.66, 1.13};
        double AngCal [] = {15, 90, -90};
        if (y < 3){
          if(navX.getYaw() > AngCal[y]+0.2 && h == 0){
            mecanum.driveCartesian( 0, 0, -0.3);
          }else if(navX.getYaw() < AngCal[y]-0.2 && h == 0){
            mecanum.driveCartesian( 0, 0, 0.3);
          }else if(h == 0){
            h ++;
            clock.reset();
          }
          if (clock.get() < TimCal[y] && h == 1){
            mecanum.driveCartesian(0, 0.3, 0);
          }
          //if(navX.getDisplacementY() < DisCal[y]/100 && h == 1){
            //mecanum.driveCartesian( 0, 0.3, 0);
          else if(h ==1 ){
            h ++;
          }
          if (h == 2){
            y ++;
            navX.reset();
            h = 0;
          } 
          System.out.println(clock.get());
          //SmartDashboard.putNumber("Displacement", navX.getDisplacementY());
        }else{
          mecanum.driveCartesian(0,0,0);
          clock.stop(); 
        }

        break; 
      case AutoNavPath:
        //double Dis [] = {63, 36.9, 32.4, 37.8, 32.4, 37.8, 63, 45, 37.8, 32.4, 28.8, 125.1, 40.5, 28.8, 117, 28.8, 138};
        double Tim [] = {1.49, 0.87, 0.77, 0.89, 0.77, 0.89, 1.49, 1.06, 0.89, 0.77, 0.68, 2,96, 0.96, 2.76, 0.68, 3.26};
        double Ang [] = {-5.9, 16, 36.7, 98.7, 81.3, 98.7, 28.1, -18.6, -101.3, -94.8, -61, -47.8, -77.1, -82, -51, 22.5, -22.5};
        
        if (x > 17){
          if(navX.getYaw() > Ang[x] && pos == 0){
            mecanum.driveCartesian( 0, 0, (-1.0));
          }
          else if (navX.getYaw() < Ang[x] && pos == 0){
            mecanum.driveCartesian( 0, 0, (1.0));
          }else{
            pos ++;
          }
          if(clock.get() < Tim[y] && pos == 1){
            mecanum.driveCartesian( 0, 0.3, 0);
          }else if (pos == 1){
            pos ++;
          }
          if(pos == 3){
            y ++;
            navX.reset();
            pos = 0;
          }
          System.out.println("Pos: "+pos+", RotZ: "+navX.getYaw()+", DisplaceY: "+navX.getDisplacementY());
        }else{
          mecanum.driveCartesian(0, 0, 0);
          clock.stop();
        }
        break;
      case AutoNavPath2:
        //double Dis2 [] = {22.4, 56.6, 44.7, 70, 36.1, 44.7, 22.4, 20, 36.1, 36.1, 20, 22.4, 44.7, 22.4, 80, 44.7, 56.6, 22.4};
        double Tim2 [] = {0.53, 1,34, 1.06, 1.65, 0.85, 1.06, 0.53, 0.47, 0.85, 0.85, 0.47, 0.53, 1.06, 0.53, 1.89, 1.06, 1.34, 0.53};
        double Ang2 [] = {-26.6, -18.4, 18.4, 18.4, 33.7, 29.7, -39.4, -26.6, -56.3, -67.4, -60.3, -26.6, -31.7, 26.6, 26.6, 14, 22.5, -22.5};
        if (z < 17){
          if(navX.getYaw() > Ang2[z]+0.2 && pos2 == 0){
            mecanum.driveCartesian( 0, 0, -0.3);
          }else if(navX.getYaw() < Ang2[z]-0.2 && pos2 == 0){
            mecanum.driveCartesian( 0, 0, 0.3);
          }else{
            pos2 ++;
          }
          if(clock.get() < Tim2[z] && pos2 ==1){
            mecanum.driveCartesian( 0, 0.3, 0);
          }else if(pos2 == 1){
            pos2 ++;
          }if(pos2 == 2){
            z ++;
            navX.reset();
            pos2 =0;
          }
        }else{
          mecanum.driveCartesian(0,0,0);
          clock.stop();
        }
        break;
      case AutoNavPath3:
      double Dis3[] = {28.3, 41.2, 76.2, 50, 28.3, 90.6, 90.6, 28.3, 30, 28.3, 90.6, 41.2, 2.24};
      double Ang3[] = {-45, -30.9, 99.2, -3.1, -90, -41.9, 167.4, -38.7, -50.2, -45, -41.9, 159.7, -30};
      double Pick [] = {0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0};
        if (g < 13){
          if(navX.getYaw() > Ang3[g]+0.2 && pos3 == 0){
            mecanum.driveCartesian( 0, 0, -0.3);
          }else if(navX.getYaw() < Ang3[g]-0.2 && pos3 == 0){
            mecanum.driveCartesian( 0, 0, 0.3);
          }else{
            pos3 ++;
          }
          if (clock.get() < Dis3[g]/42.33 && pos3 == 1){
            mecanum.driveCartesian( 0, 0.3, 0);
          }else if(pos3 == 1){
            pos ++;
          }if (pos3 == 2){
            intakewheels.set(Pick[g]);
          }else{
            pos3 ++;
          }
          if (pos3 == 3){
            g ++;
            navX.reset();
            pos3 = 0;
          }
          
        }else{
          mecanum.driveCartesian(0, 0, 0);
          clock.stop();
        }
        break;
    }
  }
  

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Timer.delay(0.02);
    mecanum.driveCartesian(joy.getX(), -1 * joy.getY(), joy.getRawAxis(5));
    shooterMech();
    intakeMech();
    climberMech();
  }
  public void shooterMech() {
    if (joy.getRawButton(T6)) {
      shootSpeed = 1;
      SmartDashboard.putBoolean("Shooter Speed", true);
    } else if (joy.getRawButton(T5)) {
      shootSpeed = 0.925;
      SmartDashboard.putBoolean("Shooter Speed", false);
    }

    if (joy.getRawButton(D)) {
      kicker.set(on);
    } else {
      kicker.set(false);
    }

    if(joy.getRawButton(Fire)){
      ShooterR.set(shootSpeed);
      ShooterL.set(-1*shootSpeed);
    }else{
      ShooterR.set(0);
      ShooterL.set(0);
    }
  }

    public void intakeMech() {
      if (joy.getRawButton(B)) {
        intakeSolenoidIn.set(on);
        intakeSolenoidOut.set(false);
      } else if (joy.getRawButton(A)) {
        intakeSolenoidIn.set(false);
        intakeSolenoidOut.set(on);
      } else {
        intakeSolenoidIn.set(false);
        intakeSolenoidOut.set(false);
      }
      if (joy.getRawButton(trigger) || joy.getRawButton(D)) {
        intakewheels.set(1);
      } else if (joy.getRawButton(pink)) {
        intakewheels.set(-1);
      } else {
        intakewheels.set(0);
      }
    }
    int armTog = 0;
    boolean arming = false;
    public void climberMech() {
      if(joy.getRawButton(E) && armTog == 0){
        armTog = 1;
      }else if(!joy.getRawButton(E) && armTog == 1){
        armTog = 2;
      }else if(joy.getRawButton(E) && armTog == 2){
        armTog = 3;
      }else if(!joy.getRawButton(E) && armTog == 3){
        armTog = 0;
      }
      if(joy.getRawButton(T1)){
        arming = true;
      }
      if ((armTog == 1 || armTog == 2) && arming) {
        climberSolenoid3.set(false);
        climberSolenoid2.set(true);
      } else {
        climberSolenoid3.set(true);
        climberSolenoid2.set(false);
      }
      if (joy.getRawButton(povUp) && tClimb.getVoltage() < 3) {
        climber.set(1);
      } else if (joy.getRawButton(povDown) && bClimb.getVoltage() < 3) {
        climber.set(-1);
      } else {
        climber.set(0);
      }
      SmartDashboard.putBoolean("Armed Climber Solenoid", arming);
      SmartDashboard.putNumber("top", tClimb.getVoltage());
    }
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
}
