package nbgy.bme.hsn.imu_native;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setClass();

        RequestUserPermission requestUserPermission = new RequestUserPermission(this);
        requestUserPermission.verifyStoragePermissions();

        final EditText editText= findViewById(R.id.editTextROS);

        Button button= findViewById(R.id.button);
        final MainActivity mainActivity= this;
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(mainActivity, StreamerService.class);
                intent.putExtra("rosIP",editText.getText().toString());
                startService(intent);
            }
        });

        // Example of a call to a native method
        //TextView tv = findViewById(R.id.sample_text);
        //tv.setText(Long.toString(getTime()));
    }


    @Override
    protected void onDestroy() {
        Intent intent = new Intent(this, StreamerService.class);
        stopService(intent);

        super.onDestroy();
    }
    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();

    public native long setClass();
}
