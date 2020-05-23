package com.example.sensorclientapp;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import static android.os.SystemClock.sleep;

public class MainActivity extends AppCompatActivity {


    EditText editText;
    Button button;
    MessageSender msg;
    private static final String TAG = "MyActivity";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        button = (Button)findViewById(R.id.Button1);
        editText = (EditText)findViewById(R.id.editText1);

        msg = new MessageSender();
        new Thread(msg).start();
    }

    public void ButtonClicked(View v)
    {
        String IpAddress = GetTextFromEditText();
        if( IpAddress != "not ip")
        {

            msg.SetIP(IpAddress, this);
            msg.Connect2Server();



            // TODO : connect to the server as a background activity

            // TODO: Initialize the sensors as different threads.

            // TODO: each time a new sensor reading is available: Add a time stamp, name of sensor and value,
            //  and add it to a list of objects of class DATAFRAME

            // TODO : Keep sending the elements in the DATAFRAME list to the server.

        }



    }

    private String GetTextFromEditText()
    {
        String s = editText.getText().toString();

        // Removes spaces, checks if the text is purely digits,
        // checks if there are 4 sets of 3 digits each seperated by "."
        s = s.trim();
        if (s.matches("[0-9.]+"))
        {
                String[] arrOfStr = s.split("\\.", 4);
                if(arrOfStr.length == 4) {
                    for (int i = 0; i < 4; i++) {
                        String str = arrOfStr[i];
                        if (str.length() > 3) {
                            s = "not ip";
                            break;
                        }
                    }
                }
                else
                    s = "not ip";
        }
        else
        {
            s = "not ip";
        }
        return s;
    }
}
