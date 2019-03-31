//
//  ff_read.h
//  ffplay
//
//  Created by wlanjie on 2018/10/22.
//  Copyright © 2018年 com.wlanjie.ffplay. All rights reserved.
//

#ifndef ff_read_h
#define ff_read_h

#include <stdio.h>
#include <pthread.h>
#include <math.h>

#define DEBUG

#ifdef DEBUG
#include <fstream>
#include <iostream>
#endif

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavutil/time.h"
#include "libavformat/avformat.h"
#include "libavutil/display.h"
#include "libavutil/eval.h"
}

#define MAX_QUEUE_SIZE (15 * 1024 * 1024)
#define MIN_FRAMES 25

#define VIDEO_PICTURE_QUEUE_SIZE 6
#define SUBPICTURE_QUEUE_SIZE 16
#define SAMPLE_QUEUE_SIZE 9
#define FRAME_QUEUE_SIZE FFMAX(SAMPLE_QUEUE_SIZE, FFMAX(VIDEO_PICTURE_QUEUE_SIZE, SUBPICTURE_QUEUE_SIZE))

#define FF_ALLOC_EVENT   (SDL_USEREVENT)
#define FF_QUIT_EVENT    (SDL_USEREVENT + 2)

/* Minimum SDL audio buffer size, in samples. */
#define SDL_AUDIO_MIN_BUFFER_SIZE 512
/* Calculate actual buffer size keeping in mind not cause too frequent audio callbacks */
#define SDL_AUDIO_MAX_CALLBACKS_PER_SEC 30

/* Step size for volume control */
#define SDL_VOLUME_STEP (SDL_MIX_MAXVOLUME / 50)

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
/* TODO: We assume that a decoded and resampled frame fits into this buffer */
#define SAMPLE_ARRAY_SIZE (8 * 65536)

/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

/* we use about AUDIO_DIFF_AVG_NB A-V differences to make the average */
#define AUDIO_DIFF_AVG_NB   20

#define CURSOR_HIDE_DELAY 1000000

/* polls for possible required screen refresh at least this often, should be less than 1/fps */
#define REFRESH_RATE 0.01

#define EXTERNAL_CLOCK_MIN_FRAMES 2
#define EXTERNAL_CLOCK_MAX_FRAMES 10

/* external clock speed adjustment constants for realtime sources based on buffer fullness */
#define EXTERNAL_CLOCK_SPEED_MIN  0.900
#define EXTERNAL_CLOCK_SPEED_MAX  1.010
#define EXTERNAL_CLOCK_SPEED_STEP 0.001

/* no AV sync correction is done if below the minimum AV sync threshold */
#define AV_SYNC_THRESHOLD_MIN 0.04
/* AV sync correction is done if above the maximum AV sync threshold */
#define AV_SYNC_THRESHOLD_MAX 0.1
/* If a frame duration is longer than this, it will not be duplicated to compensate AV sync */
#define AV_SYNC_FRAMEDUP_THRESHOLD 0.1
/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

#define USE_ONEPASS_SUBTITLE_RENDER 1

/* maximum audio speed change to get correct sync */
#define SAMPLE_CORRECTION_PERCENT_MAX 10

#define DEBUG 1

enum {
    AV_SYNC_AUDIO_MASTER, /* default choice */
    AV_SYNC_VIDEO_MASTER,
    AV_SYNC_EXTERNAL_CLOCK, /* synchronize to an external clock */
};

typedef struct MyAVPacketList {
    AVPacket pkt;
    struct MyAVPacketList *next;
    int serial;
} MyAVPacketList;

typedef struct Decoder {
    AVPacket pkt;
    AVPacket pkt_temp;
    struct PacketQueue *queue;
    AVCodecContext *avctx;
    int pkt_serial;
    int finished;
    int packet_pending;
    pthread_cond_t empty_queue_cond;
    int64_t start_pts;
    AVRational start_pts_tb;
    int64_t next_pts;
    AVRational next_pts_tb;
    pthread_t decoder_tid;
} Decoder;

typedef struct AudioParams {
    int freq;
    int channels;
    int64_t channel_layout;
    enum AVSampleFormat fmt;
    int frame_size;
    int bytes_per_sec;
} AudioParams;

typedef struct Clock {
    double speed;
    int paused;
    int *queue_serial;
    double pts;
    double last_update;
    double pts_drift;
    int serial;
} Clock;

typedef struct Frame {
    AVFrame *frame;
    double pts;
    int64_t pos;
    double duration;
    int serial;
    AVRational sar;
    AVSubtitle sub;
    AVSubtitleRect **subrects;
    int allocated;
    int reallocated;
    int width;
    int height;
    int uploaded;
    int format;
} Frame;

typedef struct PacketQueue {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int abort_request;
    int serial;
    int nb_packets;
    int size;
    MyAVPacketList *first_pkt, *last_pkt;
} PacketQueue;

typedef struct FrameQueue {
    Frame queue[FRAME_QUEUE_SIZE];
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    PacketQueue *pktq;
    int max_size;
    int keep_last;
    int rindex;
    int windex;
    int size;
    int rindex_shown;
} FrameQueue;

typedef struct VideoState {
    char *filename;
    int width;
    int height;
    int ytop;
    int xleft;
    
    Clock audclk;
    Clock vidclk;
    Clock extclk;
    
    FrameQueue pictq;
    FrameQueue subpq;
    FrameQueue sampq;
    
    PacketQueue videoq;
    PacketQueue subtitleq;
    PacketQueue audioq;
    
    pthread_cond_t continue_read_thread;
    int audio_clock_serial;
    int audio_volume;
    int muted;
    unsigned int audio_buf_index;
    unsigned int audio_buf_size;
    unsigned int audio_buf1_size;
    uint8_t *audio_buf;
    uint8_t *audio_buf1;
    int audio_write_buf_size;
    struct AudioParams audio_src;
    double audio_clock;
    double audio_hw_buf_size;
    double audio_diff_avg_coef;
    double audio_diff_threshold;
    int audio_diff_avg_count;
    double audio_diff_cum;
    int av_sync_type;
    pthread_t read_tid;
    AVStream *audio_st;
    Decoder auddec;
    
    AVStream *video_st;
    int viddec_width;
    int viddec_height;
    Decoder viddec;
    int queue_attachments_req;
    
    int video_stream;
    int audio_stream;
    int subtitle_stream;
    
    int last_video_stream;
    int last_audio_stream;
    int last_subtitle_stream;
    int eof;
    int paused;
    int last_paused;
    int read_pause_return;
    
    AVFormatContext *ic;
    AudioParams audio_filter_src;
    AudioParams audio_tgt;
    struct SwrContext *swr_ctx;
    
    int abort_request;
    double max_frame_duration;
    int realtime;
    
    double frame_last_filter_delay;
    int vfilter_idx;
    double frame_last_returned_time;
    int seek_req;
    int64_t seek_pos;
    int64_t seek_rel;
    int seek_flags;
    int step;
    double frame_timer;
    int force_refresh;
    double last_vis_time;
    int last_i_start;
    int reft_bits;
    int rdft_bits;
    int xpos;
    AVStream *subtitle_st;
    
    struct SwsContext *sub_convert_ctx;
    struct SwsContext *img_convert_ctx;
    
    int16_t sample_array[SAMPLE_ARRAY_SIZE];
    int sample_array_index;
    
    enum ShowMode {
        SHOW_MODE_NONE = -1,
        SHOW_MODE_VIDEO = 0,
        SHOW_MODE_WAVES,
        SHOW_MODE_RDFT,
        SHOW_MODE_NB
    } show_mode;
    
    class FFmpegRead* object;
} VideoState;

class FFmpegRead {
public:
    FFmpegRead();
    ~FFmpegRead();
    int init(const char* filename);
    
private:
    int frameQueueInit(FrameQueue *f, PacketQueue *pktq, int maxSize, int keepLast);
    Frame* frameQueuePeekWritable(FrameQueue* f);
    Frame* frameQueuePeekReadable(FrameQueue* f);
    void frameQueuePush(FrameQueue* f);
    Frame* frameQueuePeek(FrameQueue* f);
    int frameQueueNbRemaining(FrameQueue* f);
    Frame* frameQueuePeekLast(FrameQueue* f);
    Frame* frameQueuePeekNext(FrameQueue* f);
    void frameQueueUnrefItem(Frame* vp);
    void frameQueueNext(FrameQueue* f);
    void frameQueueSignal(FrameQueue* f);
    void frameQueueDestroy(FrameQueue* f);
    int packetQueueInit(PacketQueue* q);
    int packetQueueGet(PacketQueue* q, AVPacket* pkt, int block, int* serial);
    int packetQueuePut(PacketQueue* q, AVPacket* pkt);
    int packetQueuePutPrivate(PacketQueue* q, AVPacket* packet);
    int packetQueuePutNullPacket(PacketQueue* q, int streamIndex);
    void packetQueueStart(PacketQueue* q);
    void packetQueueAbort(PacketQueue* q);
    void packetQueueFlush(PacketQueue* q);
    void packetQueueDestroy(PacketQueue* q);
    void setClockAt(Clock* c, double pts, int serial, double time);
    void setClock(Clock* c, double pts, int serial);
    double getClock(Clock* c);
    void setClockSpeed(Clock* c, double speed);
    void syncClockToSlave(Clock* c, Clock* slave);
    void initClock(Clock* clock, int* serial);
    static int decodeInterruptCb(void* ctx);
    int getMasterSyncType(VideoState* is);
    double getMasterClock(VideoState* is);
    void streamSeek(VideoState* is, int64_t pos, int64_t rel, int seekByBytes);
    void streamTogglePause(VideoState* is);
    void stepToNextFrame(VideoState* is);
    int getVideoFrame(VideoState* is, AVFrame* frame);
    int decoderDecodeFrame(Decoder* d, AVFrame* frame, AVSubtitle* sub);
    double getRotation(AVStream* st);
    
    static void* readThread(void* arg);
    static void readThreadFailed(VideoState* is, AVFormatContext* ic, pthread_mutex_t waitMutex);
    
    VideoState* streamOpen(const char* filename);
    void streamClose(VideoState* st);
    
    static void streamComponentClose(VideoState* is, int streamIndex);
    static void decoderRelease(FFmpegRead* read, Decoder* d);
    static void decoderAbort(FFmpegRead* read, Decoder* d, FrameQueue* fq);
    static int streamComponentOpen(VideoState* is, int streamIndex, FFmpegRead* read);
    static void decoderInit(Decoder* d, AVCodecContext* avctx, PacketQueue* queue, pthread_cond_t emptyQueueCond);
    static int decoderStart(Decoder* d, void* (*fn) (void*), void* arg, FFmpegRead* read);
    static void* videoThread(void* arg);
    static void* audioThread(void* arg);
private:
    AVPacket flushPacket;
    int av_sync_type;
    int64_t startTime;
    char *wanted_stream_spec[AVMEDIA_TYPE_NB];
    int lowres;
    int fast;
    bool autoexit;
    int loop;
    int64_t duration;
    int infinite_buffer;
//    static int decoderReorderPts;
#ifdef DEBUG
    std::ofstream outputStream;
#endif
};

#endif /* ff_read_h */
