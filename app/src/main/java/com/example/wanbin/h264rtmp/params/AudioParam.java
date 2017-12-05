package com.example.wanbin.h264rtmp.params;

/**
 * Created by david on 2017/10/11.
 */

public class AudioParam {
    //采样率
    private int sampleRate = 44100;
    private int channel = 1;

    public AudioParam(int sampleRate, int channel) {
        this.sampleRate = sampleRate;
        this.channel = channel;
    }

    public AudioParam() {
    }
    public int getSampleRate() {
        return sampleRate;
    }

    public void setSampleRate(int sampleRate) {
        this.sampleRate = sampleRate;
    }

    public int getChannel() {
        return channel;
    }

    public void setChannel(int channel) {
        this.channel = channel;
    }
}
