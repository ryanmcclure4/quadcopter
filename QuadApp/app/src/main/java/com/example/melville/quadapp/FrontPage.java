package com.example.melville.quadapp;

import android.app.Activity;
import android.bluetooth.*;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

import java.io.*;
import java.util.List;
import java.util.Set;
import java.util.UUID;

// Needs more comments, hard to follow
public class FrontPage extends AppCompatActivity {

    private static final String _deviceName = "QUAD";

    TextView myLabel;
    SeekBar throttle;
    Button connectBtn;

    BluetoothAdapter mBluetoothAdapter;
    BluetoothDevice btDevice;
    BluetoothSocket btSocket;
    BluetoothProfile btProfile;

    InputStream iStream;
    OutputStream oStream;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_front_page);

        IntentFilter filter1 = new IntentFilter(BluetoothDevice.ACTION_ACL_CONNECTED);
        IntentFilter filter2 = new IntentFilter(BluetoothDevice.ACTION_ACL_DISCONNECT_REQUESTED);
        IntentFilter filter3 = new IntentFilter(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        this.registerReceiver(mReceiver, filter1);
        this.registerReceiver(mReceiver, filter2);
        this.registerReceiver(mReceiver, filter3);

        myLabel = (TextView)findViewById(R.id.label_text);
        connectBtn = (Button)findViewById(R.id.connect_btn);
        throttle = (SeekBar)findViewById(R.id.throttle);
        throttle.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                String changed = Integer.toString(progress);
                changed += "\n";
                try {
                    if(oStream != null)
                        oStream.write(changed.getBytes());
                } catch (IOException e) {
                    Log.d("BLUETEST", "Caught exception while writing to oStream : " + e.getMessage());
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        findBluetooth();
    }

    private final BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            btDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);

            // Device found
            if(BluetoothDevice.ACTION_FOUND.equals(action)){

            }
            // Device connected
            else if(BluetoothDevice.ACTION_ACL_CONNECTED.equals(action)){
                connectBtn.setBackgroundColor(Color.CYAN);
                connectBtn.setTextColor(Color.WHITE);
                connectBtn.setText("Connected");
            }
            // Device disconnected
            else if(BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action)){
                connectBtn.setBackgroundColor(Color.RED);
            }
        }
    };

    public void findBluetooth() {
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        myLabel.setText("Finding Bluetooth...");
        Log.d("BLUETEST", "Finding Bluetooth...");
        if (!mBluetoothAdapter.isEnabled())
        {
            Log.d("BLUETEST", "was not enabled");
            Intent enableBluetooth = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBluetooth, 0);
        }

        Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
        if (pairedDevices.size() > 0)
        {
            Log.d("BLUETEST", "pairedDevices.size() > 0!!!");
            for(BluetoothDevice device : pairedDevices) {
                Log.d("BLUETEST", "device name : " + device.getName());
                if (device.getName().equals(_deviceName))
                {
                    btDevice = device;
                    myLabel.setText("Found : " + _deviceName);
                    Log.d("BLUETEST", "Found : " + _deviceName);
                    break;
                }
            }
        }

    }

    public void connectBluetooth(View view) throws IOException{
        myLabel.setText("Attempting to Connect");
        Log.d("BLUETEST", "Connecting to " + btDevice.getName());
        UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
        btSocket = btDevice.createRfcommSocketToServiceRecord(uuid);
        btSocket.connect();
        oStream = btSocket.getOutputStream();
        iStream = btSocket.getInputStream();

    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
