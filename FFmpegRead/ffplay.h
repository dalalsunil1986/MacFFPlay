//
//  ffplay.h
//  ffplay
//
//  Created by wlanjie on 16/6/27.
//  Copyright © 2016年 com.wlanjie.ffplay. All rights reserved.
//

#ifndef ffplay_h
#define ffplay_h

#include "libavcodec/avcodec.h"
#include "libavutil/time.h"
#include "libavformat/avformat.h"
#include "libavfilter/avfilter.h"
#include "libavutil/opt.h"
#include "libavfilter/buffersrc.h"
#include "libavfilter/buffersink.h"
#include "libavutil/avstring.h"
#include "libswresample/swresample.h"
#ifdef __APPLE__
#include "SDL.h"
#include "libswscale/swscale.h"
#include "libavcodec/avfft.h"
#include "libavutil/display.h"
#include "libavutil/eval.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#endif

#include "pthread.h"

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

int init_ffplay(const char *filename);

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
    SDL_Texture *bmp;
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

#ifdef __APPLE__
    RDFTContext *rdft;
    FFTSample *rdft_data;
    int rdft_bits;
    int xpos;
    AVStream *subtitle_st;
    SDL_Texture *sub_texture;
    SDL_Texture *vis_texture;
#endif
    
    struct SwsContext *sub_convert_ctx;
    struct SwsContext *img_convert_ctx;
    
    AVFilterContext *in_video_filter;
    AVFilterContext *out_video_filter;
    AVFilterContext *in_audio_filter;
    AVFilterContext *out_audio_filter;
    AVFilterGraph *agraph;
    
    int16_t sample_array[SAMPLE_ARRAY_SIZE];
    int sample_array_index;
    
    enum ShowMode {
        SHOW_MODE_NONE = -1,
        SHOW_MODE_VIDEO = 0,
        SHOW_MODE_WAVES,
        SHOW_MODE_RDFT,
        SHOW_MODE_NB
    } show_mode;
} VideoState;

static char *wanted_stream_spec[AVMEDIA_TYPE_NB] = {0};
static int av_sync_type = AV_SYNC_AUDIO_MASTER;
static int64_t start_time = AV_NOPTS_VALUE; /* 如果不是AV_NOPTS_VALUE, 则从这个时间开始播放*/
static int default_width = 640;
static int default_height = 480;
static int lowres;
static int fast = 0;
static AVPacket flush_pkt;
static int64_t duration = AV_NOPTS_VALUE;
static int display_disable;
static double rdftspeed = 0.02;
static int screen_width = 0;
static int screen_height = 0;
static int is_full_screen;

static SDL_Window *window;
static SDL_Renderer *renderer;

static int64_t audio_callback_time;
#endif /* ffplay_h */
