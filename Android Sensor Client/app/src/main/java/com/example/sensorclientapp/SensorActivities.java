package com.example.sensorclientapp;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;


public class SensorActivities extends AppCompatActivity implements SensorEventListener
{
    private SensorManager sensorManager;
    private Sensor Acc, Gyr, Mag;

    private TextView textView;
    MessageSender msg;
    private Button start_button, stop_button;
    int stage_ = 0;
    private boolean start_button_activated = true;
    private boolean can_send_msg = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sensor_activities);
        start_button = (Button) findViewById(R.id.start_button);
        stop_button = (Button) findViewById(R.id.stop_button);

        msg = new MessageSender();
        new Thread(msg).start();
        textView = findViewById(R.id.textView2);
        sensorManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
        Acc = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Gyr = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Mag = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);


        sensorManager.registerListener(SensorActivities.this, Acc,SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(SensorActivities.this, Gyr,SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(SensorActivities.this, Mag,SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        Sensor s = sensorEvent.sensor;
        float[] SensorReadings;
        int sensor_type;

        if(can_send_msg)
        {
            if(stage_ == 1)
            {

                if (s.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    sensor_type = 2;
                    SensorReadings = sensorEvent.values;
                    msg.SendSensorMessage(stage_,sensor_type, SensorReadings, textView);
                }
            }

            else if(stage_ >= 2)
            {
                switch (s.getType())
                {
                    case Sensor.TYPE_ACCELEROMETER:
                        sensor_type = 0;
                        break;
                    case Sensor.TYPE_GYROSCOPE:
                        sensor_type = 1;
                        break;
                    case Sensor.TYPE_MAGNETIC_FIELD:
                        sensor_type = 2;
                        break;
                    default:
                        sensor_type = -1;

                }
                SensorReadings = sensorEvent.values;
                msg.SendSensorMessage(stage_,sensor_type, SensorReadings,textView );
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    public void StopButtonClicked(View v)
    {
        can_send_msg = false;
        if (start_button_activated)
        {
            return;
        }
        start_button_activated = true;
        switch(stage_){
            case 0:
                start_button.setText("start Magnetometer Calibration ");
                break;
            case 1:
                start_button.setText("start Initial Value Estimation");
                break;
            case 2:
                start_button.setText("Start Kalman Filter");
                break;
            default:
                start_button.setText("Error Error!");
                stop_button.setText("Error Error!");
                return;
                }
        stop_button.setText("Button Inactive");
    }

    public void StartButtonClicked(View v)
    {
        if(start_button_activated)
        {
            stage_ += 1;
            start_button.setText("Button Inactive");
            stop_button.setText("Stop Sending Messages");
            can_send_msg = true;
            start_button_activated = false;
            if(stage_ >= 4)
            {
                can_send_msg = false;
                start_button_activated = true;
                start_button.setText("Restart Magnetometer Calibration");
                stop_button.setText("button Inactive");
                stage_ = 0;
            }

        }
    }
}
