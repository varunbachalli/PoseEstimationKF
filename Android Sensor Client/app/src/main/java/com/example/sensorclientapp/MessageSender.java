package com.example.sensorclientapp;

import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.TextView;

import org.w3c.dom.Text;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;





public class MessageSender implements Runnable
{

    TextView textView;
    final String TAG =  "Message Sender";

    Handler MessageSenderHandler;
    private Context context;
    InetAddress serverAddr;
    private static Socket socket;
    int a = 0;
    private int SERVERPORT = 53798;
    private static long first_time;
    String SensorMessage;



    @Override
    public void run() {
        Log.d(TAG, "Thread Started");
        Looper.prepare();
        MessageSenderHandler = new Handler();
        Looper.loop();
    }

    public MessageSender()
    {
        first_time = System.nanoTime();
    }
    public MessageSender(String ipAddr, Context context)
    {
        this.context = context;
        try {
            serverAddr = InetAddress.getByName(ipAddr);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        Log.d(TAG, "Message Sender object created");
    }

    public void SetIP(String ipAddr, Context context)
{
    this.context = context;
    try {
        serverAddr = InetAddress.getByName(ipAddr);
    }
    catch (IOException e)
    {
        e.printStackTrace();
    }
}

    public void Connect2Server()
    {
        Log.d(TAG, "Connect to server called");
        MessageSenderHandler.post(new Runnable() {
            @Override
            public void run() {
                try {
                    socket = new Socket(serverAddr, SERVERPORT);
//                    PrintWriter pr= new PrintWriter(new BufferedWriter(
//                            new OutputStreamWriter(socket.getOutputStream())),
//                            true);
//
//                    String msg = "Intro Message from Client";
//                    pr.println(msg);
//                    pr.flush();
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (Exception e) {
                    e.printStackTrace();
                }

                if(socket!= null)
                {
                    Intent intent = new Intent(context,SensorActivities.class);
                    context.startActivity(intent);
                }
            }
        });
        Log.d(TAG, "Server Connected");
    }

    public void SendTestMessage(TextView v)
    {
        textView = v;
        a++;
        MessageSenderHandler.post(new Runnable() {
            @Override
            public void run() {
                try {
                    Handler UIhandler = new Handler(Looper.getMainLooper());
                    if (socket!= null)
                    {
                        UIhandler.post(new Runnable() {
                            @Override
                            public void run() {
                                String address = socket.getInetAddress().getHostAddress();
                                textView.setText("Socket Connected to "+ address);
                            }
                        });
                    }
//                    Thread.sleep(1000);
                    UIhandler.post(new Runnable() {
                        @Override
                        public void run() {
                            textView.setText("Send Message called"+ a);
                        }
                    });

                    PrintWriter pr;
                    pr= new PrintWriter(new BufferedWriter(
                            new OutputStreamWriter(socket.getOutputStream())),
                            true);
                    UIhandler.post(new Runnable() {
                        @Override
                        public void run() {
                            textView.setText("Print writer successfully created"+ a);
                        }
                    });
//                    Thread.sleep(1000);
                    String msg = "Test Message Message from Client"+ a;
                    pr.println(msg);
                    pr.flush();
                    UIhandler.post(new Runnable() {
                        @Override
                        public void run() {
                            textView.setText(" Message sent"+ a);
                        }
                    });
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });
    }

    public void UpdateTextView()
    {
        Handler UIHandler = new Handler(Looper.getMainLooper());
        UIHandler.post(new Runnable() {
            @Override
            public void run() {
                textView.setText(SensorMessage);
            }
        });
    }
    public void SendSensorMessage(int phase, int sensor_type, float [] SensorReadings, TextView v)
    {
        /*
        sensor_type : 0 - Acc, 1 - Gyr, 2 - Mag
         */
        textView = v;
        SensorMessage = ConvertSensorMsg(phase,sensor_type,SensorReadings);

        if(phase == 2)
        {
            if(sensor_type == 1)
            {
                UpdateTextView();
            }
        }
        else
        {
            UpdateTextView();
        }

        MessageSenderHandler.post(new Runnable() {
            @Override
            public void run() {
                PrintWriter pr;
                try {
                    pr= new PrintWriter(new BufferedWriter(
                            new OutputStreamWriter(socket.getOutputStream())),
                            true);
                    pr.println(SensorMessage);
                    pr.flush();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

    }

    public static String ConvertSensorMsg(int phase, int sensor_type, float[] SensorReadings)
    {
        String returnString = "#";
        returnString = returnString + Integer.toString(phase) + ",";
        returnString = returnString + sensor_type + ":";
        for(int i = 0; i < 3; ++i)
        {
            float f = SensorReadings[i];
            returnString+= Float.toString(f);
            returnString+=",";
        }
        long currentTimeInNano = System.nanoTime() - first_time;
        returnString+= "t:" + Long.toString(currentTimeInNano);
        returnString += String.format("%" + (99 - returnString.length()) + "s", " ");
        // set the max number of bytes to 60
        return returnString;
    }

}
