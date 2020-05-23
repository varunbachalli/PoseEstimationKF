package com.example.sensorclientapp;

import android.content.Context;
import android.content.Intent;
import android.os.AsyncTask;
import android.util.Log;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.Buffer;
import java.util.concurrent.ExecutionException;

public class ClientThread implements Runnable {

    public static Socket socket;
    public static PrintWriter pr;
    InetAddress serverAddr;
    private int SERVERPORT = 53798;
    private static final String TAG = "ClientThread";


    private Context context;

    // TODO: use Messages object here instead.
    public ClientThread(String ipAddr, Context context)  {
        this.context = context;
        try {
            serverAddr = InetAddress.getByName(ipAddr);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }

    }

    @Override
    public void run() {
        try {
            socket = new Socket(serverAddr, SERVERPORT);
            pr= new PrintWriter(new BufferedWriter(
                    new OutputStreamWriter(ClientThread.socket.getOutputStream())),
                    true);

            String msg = "Message from Client";
            pr.println(msg);
        } catch (UnknownHostException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }

        if(socket!= null)
        {
            Intent intent = new Intent(this.context,SensorActivities.class);
            context.startActivity(intent);
        }
    }


}
