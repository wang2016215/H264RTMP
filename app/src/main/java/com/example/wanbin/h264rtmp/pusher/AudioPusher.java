package com.example.wanbin.h264rtmp.pusher;

import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.util.Log;

import com.example.wanbin.h264rtmp.jni.PusherNative;
import com.example.wanbin.h264rtmp.params.AudioParam;


public class AudioPusher extends Pusher {
	private final static String TAG = "AudioPusher";
	private AudioParam mParam;
	private int minBufferSize;
	private AudioRecord audioRecord;
	private int inputSamples;

	public AudioPusher(AudioParam param, PusherNative pusherNative) {
		super(pusherNative);
		mParam = param;
		// int channel = mParam.getChannel() == 1 ? AudioFormat.CHANNEL_IN_MONO
		// : AudioFormat.CHANNEL_IN_STEREO;
		minBufferSize = AudioRecord.getMinBufferSize(mParam.getSampleRate(),
				AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT);
		audioRecord = new AudioRecord(MediaRecorder.AudioSource.MIC,
				mParam.getSampleRate(), AudioFormat.CHANNEL_IN_MONO,
				AudioFormat.ENCODING_PCM_16BIT, minBufferSize);
		System.out.println(minBufferSize);
		mNative.setAudioOptions(mParam.getSampleRate(), mParam.getChannel());
		inputSamples = mNative.getInputSamples();
		Log.d(TAG,"audio input:"+ inputSamples);
	}

	@Override
	public void startPusher() {
		if (null == audioRecord) {
			return;
		}
		mPusherRuning = true;
		if (audioRecord.getRecordingState() == AudioRecord.RECORDSTATE_STOPPED) {
			try {
				audioRecord.startRecording();
				new Thread(new AudioRecordTask()).start();
			} catch (Throwable th) {
				th.printStackTrace();
				if (null != mListener) {
					mListener.onErrorPusher(-101);
				}
			}
		}
	}

	@Override
	public void stopPusher() {
		if (null == audioRecord) {
			return;
		}
		mPusherRuning = false;
		if (audioRecord.getRecordingState() == AudioRecord.RECORDSTATE_RECORDING)
			audioRecord.stop();
	}

	@Override
	public void release() {
		if (null == audioRecord) {
			return;
		}
		mPusherRuning = false;
		if (audioRecord.getRecordingState() == AudioRecord.RECORDSTATE_STOPPED)
			audioRecord.release();
		audioRecord = null;
	}

	class AudioRecordTask implements Runnable {

		@Override
		public void run() {
			while (mPusherRuning
					&& audioRecord.getRecordingState() == AudioRecord.RECORDSTATE_RECORDING) {
				byte[] buffer = new byte[inputSamples*2];
				int len = audioRecord.read(buffer, 0, buffer.length);
				if (0 < len) {
					mNative.fireAudio(buffer, len);
				}
			}
		}
	}

}
