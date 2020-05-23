package com.example.sensorclientapp;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.widget.TextView;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.List;

import androidx.annotation.LongDef;
enum Sensor_Type
{
    NoSensor,Gyroscope, Accelerometer, Magenetometer;
}
public class Messages
{
    public static MessageSender messageSender;
    public void Connect(String ipAddr, Context context)
    {
        messageSender = new MessageSender();
        messageSender.SetIP(ipAddr,context);
        new Thread(messageSender).start();
        messageSender.Connect2Server();
    }



//
//    /*
//    Todo: Create a Static Runnable Class to send messages
//    Todo: Add a message to be sent to the message queue of the Messages looper
//     */
//
//
//    public Handler handler;
//    TextView textView;
//    volatile int a = 0;
//    public void sendSensorMessage(Sensor_Type sensor, float[] SensorReadings)
//    {
//        String msg = ConvertSensorMsg(sensor,SensorReadings);
////        handler.post(new Runnable() {
////            @Override
////            public void run() {
////                        try {
////            PrintWriter pr = new PrintWriter(ClientThread.socket.getOutputStream());
////            String msg = "Message from Client";
////            pr.print(msg);
////            pr.flush();
//////            for(int i = 0; i < 10; ++i)
//////            {
//////                String msg = "Hello" + Integer.toString(i);
//////                pr.print(msg);
//////                pr.flush();
//////            }
////        } catch (IOException e) {
////            e.printStackTrace();
////        }
////            }
////        });
//    }
//
//    // TODO : MOVE MESSAGE SENDING TO THE clientThread.java
//    public void sendTestMessage(TextView v)
//    {
//        this.textView = v;
//        a++;
//        handler.post(new Runnable() {
//            @Override
//            public void run() {
//                try {
//
//
//                    Handler UIhandler = new Handler(Looper.getMainLooper());
//                    UIhandler.post(new Runnable() {
//                        @Override
//                        public void run() {
//                            textView.setText("Send Message called"+ a);
//                        }
//                    });
//
//                    if(ClientThread.socket == null)
//                    {
//                        UIhandler.post(new Runnable() {
//                            @Override
//                            public void run() {
//                                textView.setText("socket doesn't exist"+ a);
//                            }
//                        });
//                    }
//                    Thread.sleep(1000);
//                    PrintWriter pr = new PrintWriter(new BufferedWriter(
//                            new OutputStreamWriter(ClientThread.socket.getOutputStream())),
//                            true);
//                    Thread.sleep(1000);
//                    UIhandler.post(new Runnable() {
//                        @Override
//                        public void run() {
//                            textView.setText("print writer init called"+ a);
//                        }
//                    });
//                    String msg = "Message from Client";
//                    ClientThread.pr.println(msg);
//                    Thread.sleep(1000);
//                    UIhandler.post(new Runnable() {
//                        @Override
//                        public void run() {
//                            textView.setText("message sent"+ a);
//                        }
//                    });
//
//                } catch (UnknownHostException e) {
//                    e.printStackTrace();
//                } catch (IOException e) {
//                    e.printStackTrace();
//                } catch (Exception e) {
//                    e.printStackTrace();
//                }
//            }
//        });
//    }
//
//    public static String ConvertSensorMsg(Sensor_Type sensor, float[] SensorReadings)
//    {
//
//        String returnString = "";
//        returnString = returnString + sensor.toString()+ ":";
//        for(float f : SensorReadings)
//        {
//            returnString+= Float.toString(f);
//            returnString+=",";
//        }
//        long currentTimeinNano = System.nanoTime();
//        returnString+= "t:" + Long.toString(currentTimeinNano);
//        return returnString;
//    }
//
//    @Override
//    public void run() {
//        Looper.prepare();
//        handler = new Handler();
//        Looper.loop();
//    }
}
