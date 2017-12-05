package com.example.wanbin.h264rtmp;

import android.annotation.TargetApi;
import android.hardware.Camera;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.example.wanbin.h264rtmp.pusher.LivePusher;

public class MainActivity extends AppCompatActivity implements LiveStateChangeListener ,View.OnClickListener ,SurfaceHolder.Callback {

   // private static String URL = "rtmp://59.110.240.67/myapp/mystream";

    private String URL = "rtmp://114.67.145.163/myapp/mystream";

    private Button button01;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private boolean isStart;
    private LivePusher livePusher;
    private Handler mHandler = new Handler() {
        public void handleMessage(android.os.Message msg) {
            switch (msg.what) {

                case -100:
                    Toast.makeText(MainActivity.this, "视频预览开始失败", Toast.LENGTH_SHORT).show();
                    livePusher.stopPusher();
                    break;
                case -101:
                    Toast.makeText(MainActivity.this, "音频录制失败", Toast.LENGTH_SHORT).show();
                    livePusher.stopPusher();
                    break;
                case -102:
                    Toast.makeText(MainActivity.this, "音频编码器配置失败", Toast.LENGTH_SHORT).show();
                    livePusher.stopPusher();
                    break;
                case -103:
                    Toast.makeText(MainActivity.this, "视频频编码器配置失败", Toast.LENGTH_SHORT).show();
                    livePusher.stopPusher();
                    break;
                case -104:
                    Toast.makeText(MainActivity.this, "流媒体服务器/网络等问题", Toast.LENGTH_SHORT).show();
                    livePusher.stopPusher();
                    break;
            }
            button01.setText("推流");
            isStart = false;
        };
    };

    @SuppressWarnings("deprecation")
    @TargetApi(Build.VERSION_CODES.LOLLIPOP)
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        button01 = (Button) findViewById(R.id.button_first);
        button01.setOnClickListener(this);
        findViewById(R.id.button_take).setOnClickListener(
                new View.OnClickListener() {

                    @Override
                    public void onClick(View v) {
                        livePusher.switchCamera();
                    }
                });
        mSurfaceView = (SurfaceView) this.findViewById(R.id.surface);
        mSurfaceHolder = mSurfaceView.getHolder();
        mSurfaceHolder.addCallback(this);
        livePusher = new LivePusher(this,800, 480, 1200_000, 10,
                Camera.CameraInfo.CAMERA_FACING_BACK);
        livePusher.setLiveStateChangeListener(this);
        livePusher.prepare(mSurfaceHolder);

    }




    /**
     * 可能运行在子线程
     */
    @Override
    public void onErrorPusher(int code) {
        System.out.println("code:" + code);
        mHandler.sendEmptyMessage(code);
    }




    /**
     * 可能运行在子线程
     */
    @Override
    public void onStartPusher() {
        Log.d("MainActivity", "开始推流");
    }

    /**
     * 可能运行在子线程
     */
    @Override
    public void onStopPusher() {
        Log.d("MainActivity", "结束推流");
    }

    @Override
    public void onClick(View view) {
        if (isStart) {
            button01.setText("推流");
            isStart = false;
            livePusher.stopPusher();
        } else {
            button01.setText("停止");
            isStart = true;
          //  livePusher.startPusher("rtmp://send3.douyu.com/live/3251491rYfIpscG9?wsSecret=5b2505167306127f68f9137241570d40&wsTime=59e0a540&wsSeek=off");
            livePusher.startPusher(URL);
        }
    }

    @Override
    public void surfaceCreated(SurfaceHolder surfaceHolder) {
        System.out.println("MAIN: CREATE");
    }

    @Override
    public void surfaceChanged(SurfaceHolder surfaceHolder, int i, int i1, int i2) {
        System.out.println("MAIN: CHANGE");
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder surfaceHolder) {
        System.out.println("MAIN: DESTORY");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        livePusher.relase();
    }
}
