package nbgy.bme.hsn.imu_native;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.IBinder;
import android.os.SystemClock;
import android.util.Log;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.xbw.ros.ROSClient;
import com.xbw.ros.rosbridge.ROSBridgeClient;

import java.io.StringWriter;
import java.util.HashMap;
import java.util.Map;

import androidx.annotation.Nullable;
import androidx.core.app.NotificationCompat;
import nbgy.bme.hsn.imu_native.msgs.Header;
import nbgy.bme.hsn.imu_native.msgs.Imu;
import nbgy.bme.hsn.imu_native.msgs.Time;
import nbgy.bme.hsn.imu_native.msgs.Vector3;

public class StreamerService extends Service {
    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    ROSBridgeClient client;
    //String rosServerIP="ws://192.168.0.166:9090";
    String rosServerIP="ws://18.195.124.6:9090";
    //String rosServerIP="ws://3.126.240.36:9090"; //AWS
    String TAG="StreamerService";

    SensorManager sensorManager;
    Sensor gyroscope, accelometer;

    long time, previousTime;
    Vector3 gyro;
    Vector3 accelo;

    int seq=0;
    Imu imu;
    String topic= "imu1";

    boolean acceloOK, gyroOK;
    long imuTime;

    public static final String CHANNEL_ID = "ForegroundServiceChannel";

    @Override
    public void onCreate() {

        /*
        sensorManager = (SensorManager) getSystemService (Context.SENSOR_SERVICE);
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        accelometer= sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        client = new ROSBridgeClient(rosServerIP);
        client.connect(new ROSClient.ConnectionStatusListener() {
            @Override
            public void onConnect() {
                client.setDebug(true);
                System.out.println("Client connected.");

            }

            @Override
            public void onDisconnect(boolean normal, String reason, int code) {

            }

            @Override
            public void onError(Exception ex) {
                System.out.println("Client can't connect.");
                onDestroy();
            }
        });

        advertise(topic,"sensor_msgs/Imu");
//        sensorManager.registerListener(gyroListener,gyroscope,5000);
//        sensorManager.registerListener(accelometerListener,accelometer,5000);

*/

    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        String input = intent.getStringExtra("inputExtra");
        String rosip = "ws://" + intent.getStringExtra("rosIP") +":9090";
        createNotificationChannel();
        Intent notificationIntent = new Intent(this, MainActivity.class);
        PendingIntent pendingIntent = PendingIntent.getActivity(this,
                0, notificationIntent, 0);
        Notification notification = new NotificationCompat.Builder(this, CHANNEL_ID)
                .setContentTitle("Foreground Service")
                .setContentText(input)
                .setSmallIcon(R.drawable.ic_launcher_background)
                .setContentIntent(pendingIntent)
                .build();
        startForeground(1, notification);
        //do heavy work on a background thread
        //stopSelf();
        client = new ROSBridgeClient(rosip);
        client.connect(new ROSClient.ConnectionStatusListener() {
            @Override
            public void onConnect() {
                client.setDebug(true);
                Log.i(TAG,"RosBridgeClient connected");

            }

            @Override
            public void onDisconnect(boolean normal, String reason, int code) {

            }

            @Override
            public void onError(Exception ex) {
                System.out.println("Client can't connect.");
                onDestroy();
            }
        });

        advertise(topic,"sensor_msgs/Imu");
        startImu();



        return START_NOT_STICKY;
    }

    private void createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            NotificationChannel serviceChannel = new NotificationChannel(
                    CHANNEL_ID,
                    "Foreground Service Channel",
                    NotificationManager.IMPORTANCE_DEFAULT
            );
            NotificationManager manager = getSystemService(NotificationManager.class);
            manager.createNotificationChannel(serviceChannel);
        }
    }

    @Override
    public void onDestroy() {

        unadvertise(topic);
        sensorManager.unregisterListener(gyroListener);
        sensorManager.unregisterListener(accelometerListener);
        client.disconnect();
        stopImu();
        super.onDestroy();
    }

    public void sendImuData(long timestamp, float accx,float accy, float accz, float gyrox,float gyroy, float gyroz){
        Log.i(TAG,"SEND IMU: time: "+ timestamp);
        seq++;
        long time_delta_millis = System.currentTimeMillis() - SystemClock.uptimeMillis();
        long currentTime= getTime();
//        if(Build.VERSION.SDK_INT >= 29){
//            currentTime= SystemClock.currentGnssTimeClock().millis();
//        }
//        else{
//            currentTime=System.currentTimeMillis();
//        }


        Time time=new Time(timestamp);
//        Time time2= new Time(imuTime);
//        Time time3= new Time(SystemClock.elapsedRealtimeNanos());
        Log.i("TimeImu: ", time.secs +"" +time.nsecs);

        Header header= new Header(seq,time,"imu1");

        imu= new Imu(header,new Vector3(gyrox,gyroy,gyroz),new Vector3(accx,accy,accz));

        String data= imuToJson(imu);
        String json= "{\"op\":\"publish\",\"topic\":\""+ topic +"\",\"msg\": "+data+"}";
        client.send(json);
    }

    public void advertise(String topic, String type){
        String adMsg = "{" +
                "\"op\": \"advertise\",\n" +
                "\"topic\": \"" + topic + "\",\n" +
                "\"type\": \"" + type + "\"\n" +
                "}";

        client.send(adMsg);
    }

    public void unadvertise(String topic){
        String usMsg = "{" +
                "\"op\": \"unadvertise\",\n" +
                "\"topic\": \"" + topic + "\"\n" +
                "}";

        client.send(usMsg);
    }

    public void publish(String topic, String type, Object msg) {

        Map<String, Object> jsonMsg = new HashMap<String, Object>();
        jsonMsg.put("op", "publish");
        jsonMsg.put("topic", topic);
        jsonMsg.put("type", type);
        jsonMsg.put("msg", msg);

        JsonFactory jsonFactory = new JsonFactory();
        StringWriter writer = new StringWriter();
        JsonGenerator jsonGenerator;
        ObjectMapper objectMapper = new ObjectMapper();

        try {
            jsonGenerator = jsonFactory.createGenerator(writer);
            objectMapper.writeValue(jsonGenerator, jsonMsg);
        } catch (Exception e) {
            System.out.println("Error");
        }

        String jsonMsgString = writer.toString();

        client.send(jsonMsgString);
    }

    public native long getTime();

    public native void startImu();

    public native void stopImu();

    public void sendImuMessage(){
        gyroOK=false;
        acceloOK=false;

        seq++;
        long time_delta_millis = System.currentTimeMillis() - SystemClock.uptimeMillis();
        long currentTime= getTime();
//        if(Build.VERSION.SDK_INT >= 29){
//            currentTime= SystemClock.currentGnssTimeClock().millis();
//        }
//        else{
//            currentTime=System.currentTimeMillis();
//        }


        Time time=new Time(currentTime *1000 - SystemClock.elapsedRealtimeNanos() + imuTime);
//        Time time2= new Time(imuTime);
//        Time time3= new Time(SystemClock.elapsedRealtimeNanos());
        Log.i("TimeImu: ", time.secs +"" +time.nsecs);

        Header header= new Header(seq,time,"imu1");

        imu= new Imu(header,gyro,accelo);

        String data= imuToJson(imu);
        String json= "{\"op\":\"publish\",\"topic\":\""+ topic +"\",\"msg\": "+data+"}";
        client.send(json);
    }

    public String imuToJson(Imu imu){
        String json=
                "{\n\"header\": {\n" +
                        "\t\"seq\": "+ imu.header.seq +",\n" +
                        "\t\"stamp\": {\n" +
                        "\t\t\"secs\": "+imu.header.stamp.secs +",\n" +
                        "\t\t\"nsecs\": "+imu.header.stamp.nsecs +"\n" +
                        "\t},\n" +
                        "\t\"frame_id\": \""+imu.header.frame_id+"\"\n" +
                        "},\n"+
                        "\"orientation\": {\n" +
                        "\t\"x\": "+ imu.orientation.x+ ",\n" +
                        "\t\"y\": "+imu.orientation.y+",\n" +
                        "\t\"z\": "+imu.orientation.z+",\n" +
                        "\t\"w\": "+imu.orientation.w+"\n"+
                        "},\n"+
                        "\"orientation_covariance\": [99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9],\n"+
                        "\"angular_velocity\":{\n"+
                        "\t\"x\": "+imu.angular_velocity.x+",\n"+
                        "\t\"y\": "+imu.angular_velocity.y+",\n"+
                        "\t\"z\": "+imu.angular_velocity.z+"\n"+
                        "},\n"+
                        "\"angular_velocity_covariance\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],\n"+
                        "\"linear_acceleration\": {\n"+
                        "\t\"x\": "+imu.linear_acceleration.x+",\n"+
                        "\t\"y\": "+imu.linear_acceleration.y+",\n"+
                        "\t\"z\": "+imu.linear_acceleration.z+"\n"+
                        "},\n"+
                        "\"linear_acceleration_covariance\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"+
                        "}";

        return json;

    }

    public SensorEventListener gyroListener = new SensorEventListener () {
        public void onAccuracyChanged(Sensor sensor, int acc) {
        }
        public void onSensorChanged(SensorEvent event) {
//            time=System.currentTimeMillis();
//            float elapsed= (time-previousTime) /1000.f;
//            if(elapsed==0)
//                elapsed=1;
//            float fps= (1/(elapsed));
//            previousTime= time;
            float x = event . values [0];
            float y = event . values [1];
            float z = event . values [2];
            gyro= new Vector3(x,y,z);
            gyroOK=true;
            if(acceloOK){
                sendImuMessage();
            }
            //textFPS.setText("FPS: " + fps);
            //Log.d("FPS: " , String.valueOf(fps) );
        }
    };

    public SensorEventListener accelometerListener = new SensorEventListener () {
        public void onAccuracyChanged(Sensor sensor, int acc) {
        }
        public void onSensorChanged(SensorEvent event) {
            imuTime= event.timestamp;
            float x = event . values [0];
            float y = event . values [1];
            float z = event . values [2];
            accelo= new Vector3(x,y,z);
            acceloOK=true;
            if(gyroOK){
                sendImuMessage();
            }
        }
    };
}
