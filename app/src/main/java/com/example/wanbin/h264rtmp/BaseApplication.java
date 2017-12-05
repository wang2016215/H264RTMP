package com.example.wanbin.h264rtmp;

import android.app.Application;

public class BaseApplication extends Application {

	@Override
	public void onCreate() {
		super.onCreate();
		System.out.println("A:APPLICATION");
	}
}
