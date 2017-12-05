#include <jni.h>
#include <android/log.h>

#include <pthread.h>
#include <malloc.h>

#include <x264.h>
#include <faac.h>


#include <vector>
#include "librtmp/rtmp.h"

#define DEBUG
#ifdef DEBUG
#define LOGI(...) __android_log_print(4,"NDK",__VA_ARGS__)
#define LOGD(...) __android_log_print(3,"NDK",__VA_ARGS__)
#else
#define LOGI(...)
#define LOGD(...)
#endif

extern "C" {

#ifndef ULONG
typedef unsigned long ULONG;
#endif

using namespace std;
vector<RTMPPacket *> vec;
/**
 * jni相关
 */
JavaVM *jvm;
jobject jPublisherObj;

/**
 * 	音频编码相关
 */
faacEncHandle audioEncHandle = NULL;
ULONG nInputSamples;
ULONG nMaxOutputBytes;

/**
 * 视频
 */
x264_t *videoEncHandle = NULL;
x264_picture_t *pic_in = NULL;
x264_picture_t *pic_out = NULL;
int y_len;
int u_v_len;
/**
 * rtmp处理
 */
#define _RTMP_Free(_rtmp)  if(_rtmp) {RTMP_Free(_rtmp); _rtmp = NULL;}
#define _RTMP_Close(_rtmp)  if(_rtmp ) RTMP_Close(_rtmp);

RTMP *rtmp;
char *rtmp_path;
int publishing;
int readyRtmp;
ULONG start_time;
pthread_t publisher_tid;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

static void rtmp_log_debug(int level, const char *format, va_list vl) {
#ifdef DEBUG
    FILE *fp = fopen("mnt/sdcard/120/log.txt", "a+");
    if (fp) {
        vfprintf(fp, format, vl);
        fflush(fp);
        fclose(fp);
    }
#endif
}

void throwNativeInfo(JNIEnv *env, jmethodID methodId, int code) {
    if (env && methodId && jPublisherObj) {
        env->CallVoidMethodA(jPublisherObj, methodId, (jvalue *) &code);
    }
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    jvm = vm;
    JNIEnv *env = NULL;
    jint result = -1;
    if (jvm) {
        LOGD("jvm init success");
    }
    if (vm->GetEnv((void **) &env, JNI_VERSION_1_4) != JNI_OK) {
        return result;
    }
    return JNI_VERSION_1_4;
}
void add_rtmp_packet(RTMPPacket *packet) {
    pthread_mutex_lock(&mutex);
    if (publishing && readyRtmp) {
        vec.push_back(packet);
    }
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
}

void add_aac_sequence_header() {
    if (!audioEncHandle) {
        return;
    }
    unsigned char *buf;
    ULONG len;/*buf长度,一般是2*/
    faacEncGetDecoderSpecificInfo(audioEncHandle, &buf, &len);
    RTMPPacket *packet = (RTMPPacket *) malloc(sizeof(RTMPPacket));
    RTMPPacket_Alloc(packet, len + 2);
    RTMPPacket_Reset(packet);
    unsigned char *body = (unsigned char *) packet->m_body;
    /*AF 00 + AAC RAW data*/
    body[0] = 0xAF;
    body[1] = 0x00;
    memcpy(&body[2], buf, len); /*spec_buf是AAC sequence header数据*/
    packet->m_packetType = RTMP_PACKET_TYPE_AUDIO;
    packet->m_nBodySize = len + 2;
    packet->m_nChannel = 0x04;
    packet->m_hasAbsTimestamp = 0;
    packet->m_nTimeStamp = 0;
    packet->m_hasAbsTimestamp = 0;
    packet->m_headerType = RTMP_PACKET_SIZE_LARGE;
    add_rtmp_packet(packet);
    free(buf);
}

void *publiser(void *args) {
    publishing = 1;
    LOGD("启动线程");
    JNIEnv *env;
    LOGI("Native Attach Thread");
    jvm->AttachCurrentThread(&env, 0);
    LOGI("Native Attach Thread Complete:%d", env ? 1 : 0);
    LOGI(
            "Native found Java Pubulisher Object:%d", jPublisherObj ? 1 : 0);
    jclass clazz = env->GetObjectClass(jPublisherObj);
    LOGI("Native found Java Pubulisher Class :%d", clazz ? 1 : 0);
    jmethodID errorId = env->GetMethodID(clazz, "onPostNativeError", "(I)V");
    LOGI("Native found Java Pubulisher OnError Method :%d", errorId ? 1 : 0);
    jmethodID stateId = env->GetMethodID(clazz, "onPostNativeState", "(I)V");
    LOGI("Native found Java Pubulisher onState Method :%d", stateId ? 1 : 0);
    do {
        rtmp = RTMP_Alloc();
        if (!rtmp) {
            LOGD("rtmp 初始化失败");
            throwNativeInfo(env, errorId, -104);
            goto END;
        }
        LOGI("RTMP_Alloc success");
        RTMP_Init(rtmp);
        LOGI("RTMP_Init success");
        //设置连接超时，单位秒，默认30秒
        rtmp->Link.timeout = 5;
        LOGI("RTMP_SetupURL RTMP is :%d path:%s", rtmp ? 1 : 0, rtmp_path);
        /*设置URL*/
        if (!RTMP_SetupURL(rtmp, rtmp_path)) {
            LOGD("RTMP_SetupURL() failed!");
            throwNativeInfo(env, errorId, -104);
            goto END;
        }
        LOGI("RTMP_SetupURL success");
        /*设置可写,即发布流,这个函数必须在连接前使用,否则无效*/
        RTMP_EnableWrite(rtmp);
        /*连接服务器*/
        if (!RTMP_Connect(rtmp, NULL)) {
            LOGD("RTMP_Connect() failed!");
            throwNativeInfo(env, errorId, -104);
            goto END;
        }
        LOGI("RTMP_Connect success");
        /*连接流*/
        if (!RTMP_ConnectStream(rtmp, 0)) {
            LOGD("RTMP_ConnectStream() failed!");
            throwNativeInfo(env, errorId, -104);
            goto END;
        }
        LOGI("RTMP Loop start");
        throwNativeInfo(env, stateId, 100);
        readyRtmp = 1;
        add_aac_sequence_header();
        while (publishing) {
            pthread_mutex_lock(&mutex);
            pthread_cond_wait(&cond, &mutex);
            if (!publishing) {
                pthread_mutex_unlock(&mutex);
                goto END;
            }
            if (vec.size() > 0) {
                for (vector<RTMPPacket *>::iterator it = vec.begin();
                     it != vec.end();) {
                    RTMPPacket *packet = *it;
                    packet->m_nInfoField2 = rtmp->m_stream_id;
                    int i = RTMP_SendPacket(rtmp, packet, 1);
                    if (!i) {
                        RTMPPacket_Free(packet);
                        throwNativeInfo(env, errorId, -104);
                        pthread_mutex_unlock(&mutex);
                        goto END;
                    } else {
                        it = vec.erase(it);
                        RTMPPacket_Free(packet);
                    }
                }
            }
            pthread_mutex_unlock(&mutex);
        }
        END:
        _RTMP_Close(rtmp);
        _RTMP_Free(rtmp);
    } while (0);
    readyRtmp = 0;
    publishing = 0;
    LOGD("退出线程");
    free(rtmp_path);
    rtmp_path = NULL;
    vector<RTMPPacket *>::iterator iter = vec.begin();
    for (; iter != vec.end();) {
        RTMPPacket *packet = *iter;
        if (packet) {
            RTMPPacket_Free(packet);
        }
        iter = vec.erase(iter);
    }
//	pthread_mutex_destroy(&mutex);
//	pthread_cond_destroy(&cond);
    throwNativeInfo(env, stateId, 101);
    jvm->DetachCurrentThread();
    pthread_exit(NULL);
}

void add_264_sequence_header(unsigned char *pps, unsigned char *sps,
                             int pps_len, int sps_len) {
    int body_size = 13 + sps_len + 3 + pps_len;
    RTMPPacket *packet = (RTMPPacket *) malloc(sizeof(RTMPPacket));
    RTMPPacket_Alloc(packet, body_size);
    RTMPPacket_Reset(packet);
    char *body = packet->m_body;
    int i = 0;
    body[i++] = 0x17;
    body[i++] = 0x00;
    //composition time 0x000000
    body[i++] = 0x00;
    body[i++] = 0x00;
    body[i++] = 0x00;

    /*AVCDecoderConfigurationRecord*/
    body[i++] = 0x01;
    body[i++] = sps[1];
    body[i++] = sps[2];
    body[i++] = sps[3];
    body[i++] = 0xFF;

    /*sps*/
    body[i++] = 0xE1;
    body[i++] = (sps_len >> 8) & 0xff;
    body[i++] = sps_len & 0xff;
    memcpy(&body[i], sps, sps_len);
    i += sps_len;

    /*pps*/
    body[i++] = 0x01;
    body[i++] = (pps_len >> 8) & 0xff;
    body[i++] = (pps_len) & 0xff;
    memcpy(&body[i], pps, pps_len);
    i += pps_len;

    packet->m_packetType = RTMP_PACKET_TYPE_VIDEO;
    packet->m_nBodySize = body_size;
    packet->m_nChannel = 0x04;
    packet->m_nTimeStamp = 0;
    packet->m_hasAbsTimestamp = 0;
    packet->m_headerType = RTMP_PACKET_SIZE_MEDIUM;
    add_rtmp_packet(packet);
}

void add_264_body(unsigned char *buf, int len) {
    /*去掉帧界定符 *00 00 00 01*/
    if (buf[2] == 0x00) { //
        buf += 4;
        len -= 4;
    } else if (buf[2] == 0x01) { //00 00 01
        buf += 3;
        len -= 3;
    }
    int body_size = len + 9;
    RTMPPacket *packet = (RTMPPacket *) malloc(sizeof(RTMPPacket));
    RTMPPacket_Alloc(packet, len + 9);
    char *body = packet->m_body;
    int type = buf[0] & 0x1f;
    /*key frame*/
    body[0] = 0x27;
    if (type == NAL_SLICE_IDR) {
        body[0] = 0x17;
    }
    body[1] = 0x01; /*nal unit*/
    body[2] = 0x00;
    body[3] = 0x00;
    body[4] = 0x00;

    body[5] = (len >> 24) & 0xff;
    body[6] = (len >> 16) & 0xff;
    body[7] = (len >> 8) & 0xff;
    body[8] = (len) & 0xff;

    /*copy data*/
    memcpy(&body[9], buf, len);

    packet->m_hasAbsTimestamp = 0;
    packet->m_nBodySize = body_size;
    packet->m_packetType = RTMP_PACKET_TYPE_VIDEO;
    packet->m_nChannel = 0x04;
    packet->m_headerType = RTMP_PACKET_SIZE_LARGE;
//	packet->m_nTimeStamp = -1;
    packet->m_nTimeStamp = RTMP_GetTime() - start_time;
    add_rtmp_packet(packet);
}

void add_aac_body(unsigned char *buf, int len) {
    //outputformat = 1 ADTS头 7个，写入文件
    //	outputformat = 0  直接为原始数据 不需要去掉头7个
//		buf += 7;
//		len -= 7;
    int body_size = len + 2;
    RTMPPacket *packet = (RTMPPacket *) malloc(sizeof(RTMPPacket));
    RTMPPacket_Alloc(packet, body_size);
    char *body = packet->m_body;
    /*AF 01 + AAC RAW data*/
    body[0] = 0xAF;
    body[1] = 0x01;
    memcpy(&body[2], buf, len);
    packet->m_packetType = RTMP_PACKET_TYPE_AUDIO;
    packet->m_nBodySize = body_size;
    packet->m_nChannel = 0x04;
    packet->m_hasAbsTimestamp = 0;
    packet->m_headerType = RTMP_PACKET_SIZE_MEDIUM;
//	packet->m_nTimeStamp = -1;
    packet->m_nTimeStamp = RTMP_GetTime() - start_time;
    add_rtmp_packet(packet);
}

int mWidth;
int mHeight;
int mBitrate;
int mFps;

static void x264_log_default2(void *p_unused, int i_level, const char *psz_fmt,
                              va_list arg) {

    FILE *fp = fopen("mnt/sdcard/120/log.txt", "a+");
    if (fp) {
        vfprintf(fp, psz_fmt, arg);
        fflush(fp);
        fclose(fp);
    }
}

JNIEXPORT void JNICALL Java_com_example_wanbin_h264rtmp_jni_PusherNative_setVideoOptions(
        JNIEnv *env, jobject thiz, jint width, jint height, jint bitrate,
        jint fps) {
    LOGD("视频编码参数:%dx%d %dkbs %d", width, height, mBitrate / 1000, fps);
    x264_param_t param;
    LOGD("打开视频编码器");
    //发现无法通过x264_encoder_reconfig修改分辨率。只能先关闭编码器再打开
    if (videoEncHandle) {
        LOGD("视频编码器已打开");
        if (mFps != fps || mBitrate != bitrate || mHeight != height
            || mWidth != width) {
            //属性不同
            x264_encoder_close(videoEncHandle);
            videoEncHandle = 0;
            free(pic_in);
            free(pic_out);
        } else {
            //属性相同
            return;
        }
    }
//    画面参数相关设置
    mWidth = width;
    mHeight = height;
    mBitrate = bitrate;
    mFps = fps;
    y_len = width * height;
    u_v_len = y_len / 4;
//    zerolatency 无缓存
    x264_param_default_preset(&param, "ultrafast", "zerolatency");
//	param.pf_log = x264_log_default2;
    param.i_level_idc = 51; //base_line 5.2
    param.i_csp = X264_CSP_I420;
    param.i_width = width;
    param.i_height = height;
    param.i_threads = 1;
    param.i_timebase_den = param.i_fps_num;
    param.i_timebase_num = param.i_fps_den;
    param.i_fps_num = fps; //* 帧率分子
    param.i_fps_den = 1; //* 帧率分母
    param.i_keyint_max = fps * 2;

//    码率控制相关设置
    param.rc.i_rc_method = X264_RC_ABR; //参数i_rc_method表示码率控制，CQP(恒定质量)，CRF(恒定码率)，ABR(平均码率)
    param.rc.i_bitrate = bitrate / 1000; //* 码率(比特率,单位Kbps)
    param.rc.i_vbv_max_bitrate = bitrate / 1000 * 1.2; //瞬时最大码率
    param.rc.i_vbv_buffer_size = bitrate / 1000; //设置了i_vbv_max_bitrate必须设置此参数，码率控制区大小,单位kbps

    param.b_vfr_input = 0;

    param.b_repeat_headers = 1; // 是否复制sps和pps放在每个关键帧的前面 该参数设置是让每个关键帧(I帧)都附带sps/pps。
    //baseline 满足基本要求 没有b帧 如果出现b帧可能会有跳帧
    x264_param_apply_profile(&param, "baseline");
    videoEncHandle = x264_encoder_open(&param);
    if (!videoEncHandle) {
        LOGI("视频编码器打开失败");
        jmethodID methodId = env->GetMethodID(env->GetObjectClass(thiz),
                                              "onPostNativeError", "(I)V");
        env->CallVoidMethodA(thiz, methodId, (jvalue *) -103);
        return;
    }
    LOGD("视频编码器打开完成");
    pic_in = (x264_picture_t *) malloc(sizeof(x264_picture_t));
    pic_out = (x264_picture_t *) malloc(sizeof(x264_picture_t));
    x264_picture_alloc(pic_in, X264_CSP_I420, mWidth, mHeight);
}

JNIEXPORT void JNICALL Java_com_example_wanbin_h264rtmp_jni_PusherNative_setAudioOptions(
        JNIEnv *env, jobject thiz, jint sampleRate, jint channel) {
    LOGI("打开音频编码器");
    if (audioEncHandle) { //如果已经打开 不支持修改
        LOGD("音频编码器已打开");
        return;
    }
    unsigned long nChannels = channel; // 声道数
    int nSampleRate = sampleRate;
    LOGD("音频编码参数:%d %d", nSampleRate, nChannels);
    audioEncHandle = faacEncOpen(nSampleRate, nChannels, &nInputSamples,
                                 &nMaxOutputBytes);
    if (!audioEncHandle) {
        LOGI("音频编码器打开失败");
        jmethodID methodId = env->GetMethodID(env->GetObjectClass(thiz),
                                              "onPostNativeError", "(I)V");
        env->CallVoidMethodA(thiz, methodId, (jvalue *) -102);
        return;
    }
    faacEncConfigurationPtr pConfiguration = faacEncGetCurrentConfiguration(
            audioEncHandle); //获取配置结构指针
    pConfiguration->mpegVersion = MPEG4;
    pConfiguration->allowMidside = 1;
    pConfiguration->aacObjectType = LOW;
    pConfiguration->outputFormat = 0; //输出是否包含ADTS头
    pConfiguration->useTns = 1; //时域噪音控制,大概就是消爆音
    pConfiguration->useLfe = 0;
    pConfiguration->inputFormat = FAAC_INPUT_16BIT;
    pConfiguration->quantqual = 100;
    pConfiguration->bandWidth = 0; //频宽
    pConfiguration->shortctl = SHORTCTL_NORMAL;
    if (!faacEncSetConfiguration(audioEncHandle, pConfiguration)) {
        LOGI("音频编码器配置失败");
    }
    LOGI("音频编码器打开完成");
}

JNIEXPORT void JNICALL Java_com_example_wanbin_h264rtmp_jni_PusherNative_startPusher(
        JNIEnv *env, jobject thiz, jstring url) {
    LOGD("Native start Pusher");
    if (!jPublisherObj) {
        jPublisherObj = env->NewGlobalRef(thiz);
    }
    const char *path = env->GetStringUTFChars(url, 0);
    rtmp_path = (char *) malloc(strlen(path) + 1);
    memset(rtmp_path, 0, strlen(path) + 1);
    memcpy(rtmp_path, path, strlen(path));
//	pthread_mutex_init(&mutex, NULL);
//	pthread_cond_init(&cond, NULL);
//	RTMP_LogSetCallback(rtmp_log_debug);
//	FILE *fp = fopen("mnt/sdcard/120/log1.txt", "a+");
//	RTMP_LogSetOutput(fp);

    pthread_create(&publisher_tid, NULL, publiser, NULL);
    start_time = RTMP_GetTime();
    env->ReleaseStringUTFChars(url, path);
}

JNIEXPORT void JNICALL Java_com_example_wanbin_h264rtmp_jni_PusherNative_fireAudio(
        JNIEnv *env, jobject thiz, jbyteArray buffer, jint len) {
    if (!publishing || !readyRtmp || !audioEncHandle || !rtmp
        || !RTMP_IsConnected(rtmp)) {
        return;
    }
    unsigned char *bitbuf = (unsigned char *) malloc(
            nMaxOutputBytes * sizeof(unsigned char));
    jbyte *b_buffer = env->GetByteArrayElements(buffer, 0);
    int byteslen = faacEncEncode(audioEncHandle, (int32_t *) b_buffer,
                                 nInputSamples, bitbuf, nMaxOutputBytes);
    if (byteslen > 0) {
        add_aac_body(bitbuf, byteslen);
    }
    env->ReleaseByteArrayElements(buffer, b_buffer, 0);
    if (bitbuf)
        free(bitbuf);
}

JNIEXPORT jint JNICALL Java_com_example_wanbin_h264rtmp_jni_PusherNative_getInputSamples() {
    return nInputSamples;
}

JNIEXPORT void JNICALL Java_com_example_wanbin_h264rtmp_jni_PusherNative_fireVideo(
        JNIEnv *env, jobject thiz, jbyteArray buffer) {
    if (!publishing || !readyRtmp || !videoEncHandle || !rtmp
        || !RTMP_IsConnected(rtmp)) {
        return;
    }
    jbyte *nv21_buffer = env->GetByteArrayElements(buffer, 0);
    uint8_t *u = pic_in->img.plane[1];
    uint8_t *v = pic_in->img.plane[2];
    //nv21转yuv420p  y = w*h,u/v=w*h/4
    // nv21 = yvu yuv420p=yuv y=y u=y+1+1 v=y+1
    memcpy(pic_in->img.plane[0], nv21_buffer, y_len);
    for (int i = 0; i < u_v_len; i++) {
        *(u + i) = *(nv21_buffer + y_len + i * 2 + 1);
        *(v + i) = *(nv21_buffer + y_len + i * 2);
    }
    int nNal = -1;
    x264_nal_t *nal = NULL;
    x264_picture_init(pic_out);
    LOGI("开始编码");
    if (x264_encoder_encode(videoEncHandle, &nal, &nNal, pic_in, pic_out) < 0) {
        LOGI("编码失败");
        return;
    }
    LOGI("编码完成");
    pic_in->i_pts++;
    int sps_len, pps_len;
    unsigned char sps[100];
    unsigned char pps[100];
    memset(sps, 0, 100);
    memset(pps, 0, 100);
    for (int i = 0; i < nNal; i++) {
        if (nal[i].i_type == NAL_SPS) { //sps
            sps_len = nal[i].i_payload - 4;
            memcpy(sps, nal[i].p_payload + 4, sps_len);
        } else if (nal[i].i_type == NAL_PPS) { //pps
            pps_len = nal[i].i_payload - 4;
            memcpy(pps, nal[i].p_payload + 4, pps_len);
            add_264_sequence_header(pps, sps, pps_len, sps_len);
        } else {
            /*发送普通帧(剩下全部)*/
            add_264_body(nal[i].p_payload, nal[i].i_payload);
        }
    }
    env->ReleaseByteArrayElements(buffer, nv21_buffer, 0);
}

JNIEXPORT void JNICALL Java_com_example_wanbin_h264rtmp_jni_PusherNative_stopPusher(
        JNIEnv *env, jobject thiz) {
    pthread_mutex_lock(&mutex);
    publishing = 0;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);

}

JNIEXPORT void JNICALL Java_com_example_wanbin_h264rtmp_jni_PusherNative_release(
        JNIEnv *env, jobject thiz) {
    Java_com_example_wanbin_h264rtmp_jni_PusherNative_stopPusher(env, thiz);
    if (audioEncHandle) {
        faacEncClose(audioEncHandle);
        audioEncHandle = 0;
    }
    if (videoEncHandle) {
        x264_encoder_close(videoEncHandle);
        videoEncHandle = 0;
    }
    if (jPublisherObj) {
        env->DeleteGlobalRef(jPublisherObj);
        jPublisherObj = NULL;
    }
    free(pic_in);
    free(pic_out);
}

}
