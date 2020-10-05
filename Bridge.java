package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import java.util.List;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Bridge", group = "Concept")

public class Bridge extends LinearOpMode {
         BNO055IMU imu;
         BNO055IMU imu2;
         
         
         // NormalizedColorSensor colorSensor;
             Orientation lastAngles = new Orientation();

    Orientation angles;
            double globalAngle;
           FairBotHardware robot           = new FairBotHardware();//links code to main hardware map
    private ElapsedTime runtime = new ElapsedTime();


    // values is a reference to the hsvValues array.
    //float[] hsvValues = new float[3];
    //final float values[] = hsvValues;

    // bPrevState and bCurrState keep track of the previous and current state of the button
    boolean bPrevState = false;
    boolean bCurrState = false;
double power=0;
double c_angle;
double Block1, Block2;
    // Get a reference to our sensor object.
   


@Override
    public void runOpMode() throws InterruptedException{
          robot.init(hardwareMap);
                       //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");




robot.frm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.flm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rrm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        int Distance;
         //Gyro
          BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
          
                 
        
        //GYRO
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu2.initialize(Gparameters);    
        
        //GYRO
         while (!isStopRequested() && !imu.isGyroCalibrated() || !imu2.isGyroCalibrated())
        {

        telemetry.addData("Mode", "init");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("imu2 calib status", imu2.getCalibrationStatus().toString());
        telemetry.update();
        }
        Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles3 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
       

        telemetry.addData("Mode", "ready");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("imu2 calib status", imu2.getCalibrationStatus().toString());
        telemetry.update();      
        
        
        
        
        
        
        
        
        
        
        int Block = 1;
        int seen = 0;
        
                       
                       float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        
        // wait for the start button to be pressed.
        waitForStart();
    forward(18); } 
            public void left(int Distance){
      
                   int new_d=Distance *53;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //target position
    robot.frm.setTargetPosition(-new_d);
    robot.flm.setTargetPosition(new_d);
    robot.rrm.setTargetPosition(new_d);
    robot.rlm.setTargetPosition(-new_d);
    
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rrm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rlm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set drive power
    robot.frm.setPower(-.25);
    robot.flm.setPower(-.25);
    robot.rrm.setPower(.25);
    robot.rlm.setPower(.25);
    
    while(opModeIsActive()&&robot.frm.isBusy() && robot.flm.isBusy() && robot.rrm.isBusy() && robot.rlm.isBusy()){

}
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void right(int Distance){
      
                   int new_d=Distance *53;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //target position
    robot.frm.setTargetPosition(new_d);
    robot.flm.setTargetPosition(-new_d);
    robot.rrm.setTargetPosition(-new_d);
    robot.rlm.setTargetPosition(new_d);
    
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rrm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rlm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set drive power
    robot.frm.setPower(-.25);
    robot.flm.setPower(-.25);
    robot.rrm.setPower(.25);
    robot.rlm.setPower(.25);
    
    while(opModeIsActive()&&robot.frm.isBusy() && robot.flm.isBusy() && robot.rrm.isBusy() && robot.rlm.isBusy()){
    
}
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
    
public void LE(int Lift, int Ext){
    double LP=0, EP =0;
    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.L2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    //While any motor is not at position, Keep running
    while(opModeIsActive() && ( (Math.abs(robot.lift.getCurrentPosition())< Math.abs(Lift)) || (Math.abs(robot.L2.getCurrentPosition())<Math.abs(Lift) ) || (Math.abs(robot.winch.getCurrentPosition())<Math.abs(Ext)  ))){
      
      //Run the lift
      if((Math.abs(robot.lift.getCurrentPosition())> Math.abs(Lift)) || (Math.abs(robot.L2.getCurrentPosition())>Math.abs(Lift) )){
         //If the lift has hit its spot
         LP = 0; 
      }
      else if(Lift < 0){
          //if the lift needs to go down
      LP = -1;
      }
      else if(Lift > 0){
          //if the lift needs to go up
      LP = 1;
      }
      
      
      
      //Run the extender
        if(Math.abs(robot.winch.getCurrentPosition())>Math.abs(Ext)  ){
        //If the Extender has hit its spot
         EP = 0; 
      }
      else if(Ext < 0){
          //if the Extender needs to go down
      EP = -1;
      }
      else if(Ext > 0){
          //if the Extender needs to go up
      EP = 1;
      }
      
    robot.lift.setPower(LP);
    robot.L2.setPower(LP);
    robot.winch.setPower(EP);
        
    }
   
       robot.lift.setPower(0);
    robot.L2.setPower(0);
    robot.winch.setPower(0);
   
   
   
    }
public void lift(int Distance){
    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   // robot.lift.setTargetPosition(Distance);
    //robot.L2.setTargetPosition(Distance);
    
    robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.L2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    
    
    if(Distance < 0){
    robot.lift.setPower(-1);
    robot.L2.setPower(-1);
    
    while(opModeIsActive()&&(robot.lift.getCurrentPosition()>Distance)&&(robot.L2.getCurrentPosition()>Distance)){}
    }
     else if(Distance > 0){
    robot.lift.setPower(1);
    robot.L2.setPower(1);
    
    while(opModeIsActive()&&(robot.lift.getCurrentPosition()<Distance)&&(robot.L2.getCurrentPosition()<Distance)){}
    }
    
    
   
    robot.lift.setPower(0);
    robot.L2.setPower(0);
}
public void extend(int Distance){
     robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    robot.winch.setTargetPosition(Distance);
    
    robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    if(Distance<0){
    robot.winch.setPower(-1);
    
    while(opModeIsActive()&&(robot.winch.getCurrentPosition()>Distance)){}
    }
        if(Distance>0){
    robot.winch.setPower(1);
    
    while(opModeIsActive()&&(robot.winch.getCurrentPosition()<Distance)){}
    }
    robot.winch.setPower(0);
    
}
public void Lforward(int Distance, int Lift){
      double rpower, lpower, ln, rn, power = 1, LP=0;

                   Distance =Distance *40;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    while(opModeIsActive()&&(Math.abs(robot.frm.getCurrentPosition())>0) && (Math.abs(robot.rrm.getCurrentPosition())>0) && (Math.abs(robot.flm.getCurrentPosition())>0) && (Math.abs(robot.rlm.getCurrentPosition())>0)){}
      resetAngle();
      robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.L2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
  runtime.reset();  
if(Distance >0){
 while(opModeIsActive()&&(robot.frm.getCurrentPosition()<Distance) && (robot.flm.getCurrentPosition()<Distance) && (robot.rrm.getCurrentPosition()<Distance) && (robot.rlm.getCurrentPosition()<Distance)){
 if((Math.abs(robot.lift.getCurrentPosition())> Math.abs(Lift)) || (Math.abs(robot.L2.getCurrentPosition())>Math.abs(Lift) )){
         //If the lift has hit its spot
         LP = 0; 
      }
      else if(Lift < 0){
          //if the lift needs to go down
      LP = -1;
      }
      else if(Lift > 0){
          //if the lift needs to go up
      LP = 1;
      }
    robot.lift.setPower(LP);
    robot.L2.setPower(LP);  
      
      


if( runtime.seconds() < 2){
    rpower=power * (runtime.seconds()/2);
    lpower=power * (runtime.seconds()/2);
}
else{
    rpower=power;
    lpower=power;
}
    c_angle = getAngle();
        telemetry.addData("Right", robot.frm.getCurrentPosition());
        telemetry.addData("Left", robot.flm.getCurrentPosition());
        telemetry.addData("Rear Right", robot.rrm.getCurrentPosition());
        telemetry.addData("Real Left", robot.rlm.getCurrentPosition());
        telemetry.addData("Gyro", c_angle);
    telemetry.update();
    double down = Math.abs(Distance)-(Math.abs(robot.frm.getCurrentPosition())+Math.abs(robot.flm.getCurrentPosition()))/2;
    if(down <501&&down>10){
        rpower*=(down/500);
        lpower*=(down/500);
    }
    else if(down<10){
        rpower=.2;
        lpower=.2;
        
    }
    
    if(c_angle < -0.1){
        rn = rpower+(-c_angle/10);
        ln=lpower;
    }
    else if(c_angle > 0.1){
      ln = lpower+(c_angle/10);
      rn=rpower;
    }
    else{
        rn=rpower;
        ln=lpower;
    }

          rn=rpower;
        ln=lpower;  
    robot.frm.setPower(rn);
    robot.flm.setPower(ln);
    robot.rrm.setPower(rn);
    robot.rlm.setPower(ln);
 }
}
else if(Distance<0){
   while(opModeIsActive()&&(robot.frm.getCurrentPosition()>Distance) && (robot.flm.getCurrentPosition()>Distance) && (robot.rrm.getCurrentPosition()>Distance) && (robot.rlm.getCurrentPosition()>Distance)){
if((Math.abs(robot.lift.getCurrentPosition())> Math.abs(Lift)) || (Math.abs(robot.L2.getCurrentPosition())>Math.abs(Lift) )){
         //If the lift has hit its spot
         LP = 0; 
      }
      else if(Lift < 0){
          //if the lift needs to go down
      LP = -1;
      }
      else if(Lift > 0){
          //if the lift needs to go up
      LP = 1;
      }
    robot.lift.setPower(LP);
    robot.L2.setPower(LP);  
      
      

if( runtime.seconds() < 2){
    rpower=-power * (runtime.seconds()/2);
    lpower=-power * (runtime.seconds()/2);
}
else{
    rpower=-power;
    lpower=-power;
}
    // Set drive power

    c_angle = getAngle();
     /*   telemetry.addData("Right", robot.frm.getCurrentPosition());
        telemetry.addData("Left", robot.flm.getCurrentPosition());
        telemetry.addData("Rear Right", robot.rrm.getCurrentPosition());
        telemetry.addData("Real Left", robot.rlm.getCurrentPosition());
        telemetry.addData("Gyro", c_angle);*/
    telemetry.update();
    
        double down = Math.abs(Distance)-(Math.abs(robot.frm.getCurrentPosition())+Math.abs(robot.flm.getCurrentPosition()))/2;
    if(Math.abs(down) <501&& Math.abs(down)>10){
        rpower*=(down/500);
        lpower*=(down/500);
    }
    else if(Math.abs(down)<10){
        rpower=-.2;
        lpower=-.2;
        
    }
    
    if(c_angle < -0.02){
        rn = rpower+(-c_angle/10);
        ln=lpower;
    }
    else if(c_angle > 0.02){
      ln = lpower+(c_angle/10);
      rn=rpower;
    }
    else{
        rn=rpower;
        ln=lpower;
    }
    
    
            rn=rpower;
        ln=lpower;
   robot.frm.setPower(rn);
    robot.flm.setPower(ln);
    robot.rrm.setPower(rn);
    robot.rlm.setPower(ln);
 }
   
}
    
    
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);
   
                      
    }
public void forward(int Distance){
      double rpower, lpower, ln, rn, power = .4;

                   Distance =Distance *40;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    while(opModeIsActive()&&(Math.abs(robot.frm.getCurrentPosition())>0) && (Math.abs(robot.rrm.getCurrentPosition())>0) && (Math.abs(robot.flm.getCurrentPosition())>0) && (Math.abs(robot.rlm.getCurrentPosition())>0)){}
      resetAngle();

    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
  runtime.reset();  
if(Distance >0){
 while(opModeIsActive()&&(robot.frm.getCurrentPosition()<Distance) && (robot.flm.getCurrentPosition()<Distance) && (robot.rrm.getCurrentPosition()<Distance) && (robot.rlm.getCurrentPosition()<Distance)){
if( runtime.seconds() < 2){
    rpower=power * (runtime.seconds()/2);
    lpower=power * (runtime.seconds()/2);
}
else{
    rpower=power;
    lpower=power;
}
    c_angle = getAngle();
        telemetry.addData("Right", robot.frm.getCurrentPosition());
        telemetry.addData("Left", robot.flm.getCurrentPosition());
        telemetry.addData("Rear Right", robot.rrm.getCurrentPosition());
        telemetry.addData("Real Left", robot.rlm.getCurrentPosition());
        telemetry.addData("Gyro", c_angle);
    telemetry.update();
    double down = Math.abs(Distance)-(Math.abs(robot.frm.getCurrentPosition())+Math.abs(robot.flm.getCurrentPosition()))/2;
    if(down <301&&down>10){
        rpower*=(down/300);
        lpower*=(down/300);
    }
    else if(down<10){
        rpower=.2;
        lpower=.2;
        
    }
    
    if(c_angle < -0.1){
        rn = rpower+(-c_angle/10);
        ln=lpower;
    }
    else if(c_angle > 0.1){
      ln = lpower+(c_angle/10);
      rn=rpower;
    }
    else{
        rn=rpower;
        ln=lpower;
    }

          rn=rpower;
        ln=lpower;  
    robot.frm.setPower(rn);
    robot.flm.setPower(ln);
    robot.rrm.setPower(rn);
    robot.rlm.setPower(ln);
 }
}
else if(Distance<0){
   while(opModeIsActive()&&(robot.frm.getCurrentPosition()>Distance) && (robot.flm.getCurrentPosition()>Distance) && (robot.rrm.getCurrentPosition()>Distance) && (robot.rlm.getCurrentPosition()>Distance)){
if( runtime.seconds() < 2){
    rpower=-power * (runtime.seconds()/2);
    lpower=-power * (runtime.seconds()/2);
}
else{
    rpower=-power;
    lpower=-power;
}
    // Set drive power

    c_angle = getAngle();
     /*   telemetry.addData("Right", robot.frm.getCurrentPosition());
        telemetry.addData("Left", robot.flm.getCurrentPosition());
        telemetry.addData("Rear Right", robot.rrm.getCurrentPosition());
        telemetry.addData("Real Left", robot.rlm.getCurrentPosition());
        telemetry.addData("Gyro", c_angle);*/
    telemetry.update();
    
        double down = Math.abs(Distance)-(Math.abs(robot.frm.getCurrentPosition())+Math.abs(robot.flm.getCurrentPosition()))/2;
    if(Math.abs(down) <601&& Math.abs(down)>300&&Math.abs(((down-300)/300))>.2){
        rpower*=((down-300)/300);
        lpower*=((down-300)/300);
    }
    else if(Math.abs(down)<299||Math.abs(((down-300)/300))<.2){
        rpower=-.2;
        lpower=-.2;
        
    }
    
    if(c_angle < -0.02){
        rn = rpower+(-c_angle/10);
        ln=lpower;
    }
    else if(c_angle > 0.02){
      ln = lpower+(c_angle/10);
      rn=rpower;
    }
    else{
        rn=rpower;
        ln=lpower;
    }
    
    
            rn=rpower;
        ln=lpower;
   robot.frm.setPower(rn);
    robot.flm.setPower(ln);
    robot.rrm.setPower(rn);
    robot.rlm.setPower(ln);
 }
   
}
    
    
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);
   
                      
    }
public void turn(int Distance){
     resetAngle();
    double rpower, lpower, ln, rn;
    double power = .8;
      runtime.reset();
                   Distance=Distance *10;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    if (Distance > 0){
        
    
    // Set drive power

    
    while(opModeIsActive()&&(-robot.frm.getCurrentPosition()<Distance) && (robot.flm.getCurrentPosition()<Distance) && (-robot.rrm.getCurrentPosition()<Distance) && (robot.rlm.getCurrentPosition()<Distance)){
if( runtime.seconds() < 2){
    rpower=power * (runtime.seconds()/2);
    lpower=power * (runtime.seconds()/2);
}
else{
    rpower=power;
    lpower=power;
}
        double down = Math.abs(Distance)-(Math.abs(robot.frm.getCurrentPosition())+Math.abs(robot.flm.getCurrentPosition()))/2;
    if(Math.abs(down) <300&&Math.abs(down)>10){
        rpower*=(Math.abs(down)/300);
        lpower*=(Math.abs(down)/300);
    }
        else if(Math.abs(down)<10){
        rpower=.15;
        lpower=.15;
        
    }
    
    robot.frm.setPower(-rpower);
    robot.flm.setPower(lpower);
    robot.rrm.setPower(-rpower);
    robot.rlm.setPower(lpower);
}}

    else if (Distance < 0){
        
    

    
    while(opModeIsActive()&&(-robot.frm.getCurrentPosition()>Distance) && (robot.flm.getCurrentPosition()>Distance) && (robot.rrm.getCurrentPosition()>Distance) && (-robot.rlm.getCurrentPosition()>Distance)){
if( runtime.seconds() < 2){
    rpower=power * (runtime.seconds()/2);
    lpower=power * (runtime.seconds()/2);
}
else{
    rpower=power;
    lpower=power;
}
        double down = Math.abs(Distance)-(Math.abs(robot.frm.getCurrentPosition())+Math.abs(robot.flm.getCurrentPosition()))/2;
    if(Math.abs(down) <300&&Math.abs(down)>10){
        rpower*=(Math.abs(down)/300);
        lpower*=(Math.abs(down)/300);
    }
        else if(Math.abs(down)<10){
        rpower=.15;
        lpower=.15;
        
    }
    robot.frm.setPower(rpower);
    robot.flm.setPower(-lpower);
    robot.rrm.setPower(rpower);
    robot.rlm.setPower(-lpower);
}}


    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
   public void turng(int Distance){

      double cDist = 0;
                   resetAngle();
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
   while(opModeIsActive() && Math.abs(cDist) < Math.abs(Distance)){
       power = .5;
if( runtime.seconds() < 1){
    power*= (runtime.seconds()/1);
}
else{
    power=power;
}
    cDist = getAngle();

        telemetry.addData("Gyro", cDist);
    telemetry.update();
        double down = Math.abs(Distance)-Math.abs(cDist);
    if(down <40&&down>10&& power*((down-10)/30)>.15){
        power*=((down-10)/30);
    }
    else if(down<10 || power*((down-10)/30)<.15){
        power=.15;
        
    }
       
       
    if (Distance > 0){
    // Set drive power
    robot.frm.setPower(-power);
    robot.flm.setPower(power);
    robot.rrm.setPower(-power);
    robot.rlm.setPower(power);
    }
    else if (Distance < 0){
    // Set drive power
    robot.frm.setPower(power);
    robot.flm.setPower(-power);
    robot.rrm.setPower(power);
    robot.rlm.setPower(-power);
}
}

    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void resetAngle() {
        Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        lastAngles = angles2;
        globalAngle = 0;
    }
    public double getAngle()   {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

    Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        
        double deltaAngle = angles2.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles2;

        return globalAngle;
    }
}
