package com.example.wanbin.h264rtmp.pusher;


import com.example.wanbin.h264rtmp.LiveStateChangeListener;
import com.example.wanbin.h264rtmp.jni.PusherNative;

public abstract class Pusher {

	protected boolean mPusherRuning;
	protected PusherNative mNative;
	protected LiveStateChangeListener mListener;

	public Pusher(PusherNative pusherNative) {
		mNative = pusherNative;
	}

	public void setLiveStateChangeListener(LiveStateChangeListener listener) {
		mListener = listener;
	}

	public abstract void startPusher();

	public abstract void stopPusher();

	public abstract void release();
}
