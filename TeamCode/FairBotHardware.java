package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;

 
public class FairBotHardware
{
    /* Public OpMode members. */
    //public DcMotor intake = null;
    //public DcMotor intake2 = null;
    //public DcMotor  dumper   = null;
    public DcMotor frm = null;
    public DcMotor flm = null;
    public DcMotor rrm = null;
    public DcMotor rlm = null;
    public DcMotor lift = null;
    public DcMotor L2 = null;
    public DcMotor winch = null;
    DigitalChannel TouchHigh;
    DigitalChannel TouchLow;
    public Servo claw = null;
    public Servo base = null;
    public Servo base2 = null;
     ColorSensor sensorColor;
     ColorSensor sensorColor2; 
    DistanceSensor sensorDistance;

    /*public DcMotor winch = null;
    public DcMotor l_arm = null;
    public DcMotor r_arm = null;
    public DcMotor sweep = null;
    public Servo color = null;
    public Servo hang = null;
    public Servo fold = null;
    public Servo marker = null;*/
    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
  
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //intake2 = hwMap.get(DcMotor.class, "in2");
        //intake = hwMap.get(DcMotor.class, "in");
        //dumper  = hwMap.get(DcMotor.class, "dump");
        frm = hwMap.get(DcMotor.class, "fr");
        flm = hwMap.get(DcMotor.class, "fl");
        rrm = hwMap.get(DcMotor.class, "rr");
        rlm = hwMap.get(DcMotor.class, "rl");
        lift = hwMap.get(DcMotor.class, "lift");
        L2 = hwMap.get(DcMotor.class, "L2");
        TouchHigh = hwMap.get(DigitalChannel.class, "TouchHigh");
        TouchLow = hwMap.get(DigitalChannel.class, "TouchLow");
        claw = hwMap.get(Servo.class, "claw");
        base = hwMap.get(Servo.class, "base");
        base2 = hwMap.get(Servo.class, "base2");
        winch = hwMap.get(DcMotor.class, "winch");
       /* winch = hwMap.get(DcMotor.class, "winch");
        l_arm = hwMap.get(DcMotor.class, "left_arm");
        r_arm = hwMap.get(DcMotor.class, "right_arm");
        sweep = hwMap.get(DcMotor.class, "sweep");*/
        //intake2.setDirection(DcMotor.Direction.REVERSE);
        frm.setDirection(DcMotor.Direction.REVERSE);
        rrm.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        //r_arm.setDirection(DcMotor.Direction.REVERSE);
        //dumper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         sensorColor = hwMap.get(ColorSensor.class, "color");
         sensorColor2 = hwMap.get(ColorSensor.class, "color2");
sensorDistance = hwMap.get(DistanceSensor.class, "color");

        frm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
              
        //winch.setDirection(DcMotor.Direction.FORWARD);

        /*color = hwMap.get(Servo.class, "color");
        hang = hwMap.get(Servo.class, "hang");
        fold = hwMap.get(Servo.class, "fold");
         marker = hwMap.get(Servo.class, "marker");
        // fold.setPosition(.32);
         color.setPosition(.7);
                 marker.setPosition(.6);*/
    }
public void DriveForward(double power)
{
    frm.setPower(power);
    flm.setPower(power);
    rrm.setPower(power);
    rlm.setPower(power);
}
public void DriveForwardTime (double power, long time)throws InterruptedException{ 
    DriveForward(power);
    Thread.sleep(time);
}
public void DriveForwardDistance(double power, int distance)
{
    // Reset encoders
    int new_d=distance*44;
    frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //target position
    frm.setTargetPosition(new_d);
    flm.setTargetPosition(new_d);
    rrm.setTargetPosition(new_d);
    rlm.setTargetPosition(new_d);
    
    //set to Run_TO_POSITION OpMode
    frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rrm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rlm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set drive power
    DriveForward(power);
    
    while(frm.isBusy() && flm.isBusy() && rrm.isBusy() && rlm.isBusy()){
    
}
}

public void TLDistance(double power, Double distance)
{
    // Reset encoders
    Double new_d=distance*44;
    int n_d = new_d.intValue();
    frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //target position
    frm.setTargetPosition(n_d);
    flm.setTargetPosition(-n_d);
    rrm.setTargetPosition(n_d);
    rlm.setTargetPosition(-n_d);
    
    //set to Run_TO_POSITION OpMode
    frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rrm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rlm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set drive power
    TurnLeft(power);
    
    while(frm.isBusy() && flm.isBusy() && rrm.isBusy() && rlm.isBusy()){
    
}
}

public void StopDriving()
{
    DriveForward(0);

}

public void TurnLeft(double power)
{
    frm.setPower(power);
    flm.setPower(-power);
    rrm.setPower(power);
    rlm.setPower(-power);
}


public void TurnLeftTime(double power, long time)throws InterruptedException
{
    TurnLeft(power);
    Thread.sleep(time);
}
}
