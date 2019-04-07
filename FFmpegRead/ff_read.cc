//
//  ff_read.cc
//  ffplay
//
//  Created by wlanjie on 2018/10/22.
//  Copyright © 2018年 com.wlanjie.ffplay. All rights reserved.
//

#include "ff_read.h"
#include<sys/time.h>
#include <inttypes.h>
#include <math.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>

FFmpegRead::FFmpegRead() {
    av_sync_type = AV_SYNC_AUDIO_MASTER;
    startTime = AV_NOPTS_VALUE;
    for (int i = 0; i < AVMEDIA_TYPE_NB; i++) {
        wanted_stream_spec[i] = nullptr;
    }
//    memset(wanted_stream_spec, 0, AVMEDIA_TYPE_NB);
    lowres = 0;
    fast = 0;
    duration = AV_NOPTS_VALUE;
    infinite_buffer = -1;
}

FFmpegRead::~FFmpegRead() {
    
}

int FFmpegRead::frameQueueInit(FrameQueue *f, PacketQueue *pktq, int maxSize, int keepLast) {
    memset(f, 0, sizeof(FrameQueue));
    int result = pthread_mutex_init(&f->mutex, nullptr);
    if (result != 0) {
        av_log(nullptr, AV_LOG_FATAL, "Frame Init mutex error");
        return AVERROR(ENOMEM);
    }
    result = pthread_cond_init(&f->cond, nullptr);
    if (result != 0) {
        av_log(nullptr, AV_LOG_FATAL, "Frame Init cond error");
        return AVERROR(ENOMEM);
    }
    f->pktq = pktq;
    f->max_size = maxSize;
    f->keep_last = keepLast;
    for (int i = 0; i < f->max_size; i++) {
        if (!(f->queue[i].frame = av_frame_alloc())) {
            return AVERROR(ENOMEM);
        }
    }
    return 0;
}

Frame* FFmpegRead::frameQueuePeekWritable(FrameQueue* f) {
    pthread_mutex_lock(&f->mutex);
    while (f->size >= f->max_size && !f->pktq->abort_request) {
        pthread_cond_wait(&f->cond, &f->mutex);
    }
    pthread_mutex_unlock(&f->mutex);
    if (f->pktq->abort_request) {
        return nullptr;
    }
    return &f->queue[f->windex];
}

Frame* FFmpegRead::frameQueuePeekReadable(FrameQueue* f) {
    pthread_mutex_lock(&f->mutex);
    while (f->size - f->rindex_shown <= 0 && !f->pktq->abort_request) {
        pthread_cond_wait(&f->cond, &f->mutex);
    }
    pthread_mutex_unlock(&f->mutex);
    if (f->pktq->abort_request) {
        return nullptr;
    }
    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

void FFmpegRead::frameQueuePush(FrameQueue* f) {
    if (++f->windex == f->max_size) {
        f->windex = 0;
    }
    pthread_mutex_lock(&f->mutex);
    f->size++;
    pthread_cond_signal(&f->cond);
    pthread_mutex_unlock(&f->mutex);
}

Frame* FFmpegRead::frameQueuePeek(FrameQueue* f) {
    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

int FFmpegRead::frameQueueNbRemaining(FrameQueue* f) {
    return f->size - f->rindex_shown;
}

Frame* FFmpegRead::frameQueuePeekLast(FrameQueue* f) {
    return &f->queue[f->rindex];
}

Frame* FFmpegRead::frameQueuePeekNext(FrameQueue* f) {
    return &f->queue[(f->rindex + f->rindex_shown + 1) % f->max_size];
}

void FFmpegRead::frameQueueUnrefItem(Frame* vp) {
    av_frame_unref(vp->frame);
}

void FFmpegRead::frameQueueNext(FrameQueue* f) {
    if (f->keep_last && !f->rindex_shown) {
        f->rindex_shown = 1;
        return;
    }
    frameQueueUnrefItem(&f->queue[f->rindex]);
    if (++f->rindex == f->max_size) {
        f->rindex = 0;
    }
    pthread_mutex_lock(&f->mutex);
    f->size--;
    pthread_cond_signal(&f->cond);
    pthread_mutex_unlock(&f->mutex);
}

void FFmpegRead::frameQueueSignal(FrameQueue* f) {
    pthread_mutex_lock(&f->mutex);
    pthread_cond_signal(&f->cond);
    pthread_mutex_unlock(&f->mutex);
}

void FFmpegRead::frameQueueDestroy(FrameQueue* f) {
    for (int i = 0; i < f->max_size; i++) {
        Frame* vp = &f->queue[i];
        frameQueueUnrefItem(vp);
        av_frame_free(&vp->frame);
    }
    pthread_mutex_destroy(&f->mutex);
    pthread_cond_destroy(&f->cond);
}

int FFmpegRead::packetQueueInit(PacketQueue* q) {
    memset(q, 0, sizeof(PacketQueue));
    int result = pthread_mutex_init(&q->mutex, nullptr);
    if (result != 0) {
        av_log(nullptr, AV_LOG_FATAL, "Packet Init create mutex error.");
        return AVERROR(ENOMEM);
    }
    result = pthread_cond_init(&q->cond, nullptr);
    if (result != 0) {
        av_log(nullptr, AV_LOG_FATAL, "Packet Init create cond error.");
        return AVERROR(ENOMEM);
    }
    q->abort_request = 1;
    return 0;
}

int FFmpegRead::packetQueueGet(PacketQueue *q, AVPacket *pkt, int block, int *serial) {
    MyAVPacketList* pkt1;
    int ret;
    pthread_mutex_lock(&q->mutex);
    for (;;) {
        if (q->abort_request) {
            ret = -1;
            break;
        }
        pkt1 = q->first_pkt;
        if (pkt1) {
            q->first_pkt = pkt1->next;
            if (!q->first_pkt) {
                q->last_pkt = nullptr;
            }
            q->nb_packets--;
            q->size -= pkt1->pkt.size + sizeof(*pkt1);
            *pkt = pkt1->pkt;
            if (serial) {
                *serial = pkt1->serial;
            }
            av_free(pkt1);
            ret = 1;
            break;
        } else if (!block) {
            ret = 0;
            break;
        } else {
            pthread_cond_wait(&q->cond, &q->mutex);
        }
    }
    pthread_mutex_unlock(&q->mutex);
    return ret;
}

int FFmpegRead::packetQueuePut(PacketQueue *q, AVPacket *pkt) {
    int ret;
    pthread_mutex_lock(&q->mutex);
    ret = packetQueuePutPrivate(q, pkt);
    pthread_mutex_unlock(&q->mutex);
    if (pkt != &flushPacket && ret < 0) {
        av_packet_unref(pkt);
    }
    return ret;
}

int FFmpegRead::packetQueuePutPrivate(PacketQueue* q, AVPacket* packet) {
    if (q->abort_request) {
        return -1;
    }
    MyAVPacketList* pkt1 = reinterpret_cast<MyAVPacketList*>(av_malloc(sizeof(MyAVPacketList)));
    if (pkt1 == nullptr) {
        return AVERROR(ENOMEM);
    }
    pkt1->pkt = *packet;
    pkt1->next = nullptr;
    if (packet == &flushPacket) {
        q->serial++;
    }
    pkt1->serial = q->serial;
    if (!q->last_pkt) {
        q->first_pkt = pkt1;
    } else{
        q->last_pkt->next = pkt1;
    }
    q->last_pkt = pkt1;
    q->nb_packets++;
    q->size += pkt1->pkt.size + sizeof(*pkt1);
    pthread_cond_signal(&q->cond);
    return 0;
}

int FFmpegRead::packetQueuePutNullPacket(PacketQueue *q, int streamIndex) {
    AVPacket pkt1, *pkt = &pkt1;
    av_init_packet(pkt);
    pkt->data = nullptr;
    pkt->size = 0;
    pkt->stream_index = streamIndex;
    return packetQueuePut(q, pkt);
}

void FFmpegRead::packetQueueStart(PacketQueue* q) {
    pthread_mutex_lock(&q->mutex);
    q->abort_request = 0;
    packetQueuePutPrivate(q, &flushPacket);
    pthread_mutex_unlock(&q->mutex);
}

void FFmpegRead::packetQueueAbort(PacketQueue* q) {
    pthread_mutex_lock(&q->mutex);
    q->abort_request = 1;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
}

void FFmpegRead::packetQueueFlush(PacketQueue* q) {
    MyAVPacketList *pkt, *pkt1;
    pthread_mutex_lock(&q->mutex);
    for (pkt = q->first_pkt; pkt; pkt = pkt1) {
        pkt1 = pkt->next;
        av_packet_unref(&pkt->pkt);
        av_freep(&pkt);
    }
    q->last_pkt = nullptr;
    q->first_pkt = nullptr;
    q->nb_packets = 0;
    q->size = 0;
    pthread_mutex_unlock(&q->mutex);
}

void FFmpegRead::packetQueueDestroy(PacketQueue* q) {
    packetQueueFlush(q);
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond);
}

void FFmpegRead::setClockAt(Clock* c, double pts, int serial, double time) {
    c->pts = pts;
    c->last_update = time;
    c->pts_drift = c->pts - time;
    c->serial = serial;
}

void FFmpegRead::setClock(Clock* c, double pts, int serial) {
    double time = av_gettime_relative() / 1000000.0;
    setClockAt(c, pts, serial, time);
}

double FFmpegRead::getClock(Clock* c) {
    if (*c->queue_serial != c->serial) {
        return NAN;
    }
    if (c->paused) {
        return c->pts;
    } else {
        double time = av_gettime_relative() / 1000000.0;
        return c->pts_drift + time - (time - c->last_update) * (1.0 - c->speed);
    }
}

void FFmpegRead::setClockSpeed(Clock* c, double speed) {
    setClock(c, getClock(c), c->serial);
    c->speed = speed;
}

void FFmpegRead::syncClockToSlave(Clock* c, Clock* slave) {
    double clock = getClock(c);
    double slaveClock = getClock(slave);
    // TODO isnan macos
    if (!isnan(slaveClock) && (isnan(clock) || fabs(clock - slaveClock) > AV_NOSYNC_THRESHOLD)) {
        setClock(c, slaveClock, slave->serial);
    }
}

void FFmpegRead::initClock(Clock* clock, int* serial) {
    clock->speed = 1.0;
    clock->paused = 0;
    clock->queue_serial = serial;
    setClock(clock, NAN, -1);
}

int FFmpegRead::decodeInterruptCb(void* ctx) {
    VideoState *is = reinterpret_cast<VideoState*>(ctx);
    return is->abort_request;
    
}

int FFmpegRead::getMasterSyncType(VideoState* is) {
    if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
        if (is->video_st) {
            return AV_SYNC_VIDEO_MASTER;
        } else {
            return AV_SYNC_AUDIO_MASTER;
        }
    } else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
        if (is->audio_st) {
            return AV_SYNC_AUDIO_MASTER;
        } else {
            return AV_SYNC_EXTERNAL_CLOCK;
        }
    } else {
        return AV_SYNC_EXTERNAL_CLOCK;
    }
}

double FFmpegRead::getMasterClock(VideoState* is) {
    double val;
    switch (getMasterSyncType(is)) {
        case AV_SYNC_VIDEO_MASTER:
            val = getClock(&is->vidclk);
            break;
            
        case AV_SYNC_AUDIO_MASTER:
            val = getClock(&is->audclk);
            break;
        default:
            val = getClock(&is->extclk);
            break;
    }
    return val;
}

void FFmpegRead::streamSeek(VideoState *is, int64_t pos, int64_t rel, int seekByBytes) {
    if (!is->seek_req) {
        is->seek_pos = pos;
        is->seek_rel = rel;
        is->seek_flags &= ~AVSEEK_FLAG_BYTE;
        if (seekByBytes) {
            is->seek_flags |= AVSEEK_FLAG_BYTE;
        }
        is->seek_req = 1;
        pthread_cond_signal(&is->continue_read_thread);
    }
}

void FFmpegRead::streamTogglePause(VideoState *is) {
    if (is->paused) {
        is->frame_timer += av_gettime_relative() / 1000000.0 - is->vidclk.last_update;
        if (is->read_pause_return != AVERROR(ENOSYS)) {
            is->vidclk.paused = 0;
        }
        setClock(&is->extclk, getClock(&is->vidclk), is->vidclk.serial);
    }
    setClock(&is->extclk, getClock(&is->extclk), is->extclk.serial);
    is->paused = is->audclk.paused = is->vidclk.paused = is->extclk.paused = !is->paused;
}

void FFmpegRead::stepToNextFrame(VideoState *is) {
    if (is->paused) {
        streamTogglePause(is);
    }
    is->step = 1;
}

void FFmpegRead::decoderInit(Decoder *d, AVCodecContext *avctx, PacketQueue *queue, pthread_cond_t emptyQueueCond) {
    memset(d, 0, sizeof(Decoder));
    d->avctx = avctx;
    d->queue = queue;
    d->empty_queue_cond = emptyQueueCond;
    d->start_pts = AV_NOPTS_VALUE;
}

int FFmpegRead::decoderStart(Decoder *d, void* (*fn)(void *), void *arg, FFmpegRead* read) {
    read->packetQueueStart(d->queue);
    int result = pthread_create(&d->decoder_tid, nullptr, fn, arg);
    if (result != 0) {
        return result;
    }
    return 0;
}

void* FFmpegRead::videoThread(void *arg) {
    VideoState* is = reinterpret_cast<VideoState*>(arg);
    int ret;
    AVFrame* frame = av_frame_alloc();
    double pts;
    double duration;
    AVRational tb = is->video_st->time_base;
    AVRational frameRate = av_guess_frame_rate(is->ic, is->video_st, nullptr);
    
    int lastSerial = -1;
    if (frame == nullptr) {
        return nullptr;
    }
    while (1) {
        ret = is->object->getVideoFrame(is, frame);
        if (ret < 0) {
            return 0;
        }
        if (!ret) {
            continue;
        }
        is->frame_last_filter_delay = av_gettime_relative() / 1000000.0 - is->frame_last_returned_time;
        if (fabs(is->frame_last_filter_delay) > AV_NOSYNC_THRESHOLD / 10.0) {
            is->frame_last_filter_delay = 0;
        }
        
    }
    return nullptr;
}

int FFmpegRead::decoderDecodeFrame(Decoder *d, AVFrame *frame, AVSubtitle *sub) {
    int gotFrame = 0;
    do {
        int ret = -1;
        if (d->queue->abort_request) {
            return -1;
        }
        if (!d->packet_pending || d->queue->serial != d->pkt_serial) {
            AVPacket pkt;
            do {
                if (d->queue->nb_packets == 0) {
                    pthread_cond_signal(&d->empty_queue_cond);
                }
                if (packetQueueGet(d->queue, &pkt, 1, &d->pkt_serial) < 0) {
                    return -1;
                }
                if (pkt.data == flushPacket.data) {
                    avcodec_flush_buffers(d->avctx);
                    d->finished = 0;
                    d->next_pts = d->start_pts;
                    d->next_pts_tb = d->start_pts_tb;
                }
                
            } while (pkt.data == flushPacket.data || d->queue->serial != d->pkt_serial);
            av_packet_unref(&d->pkt);
            d->pkt_temp = d->pkt = pkt;
            d->packet_pending = 1;
        }
        switch (d->avctx->codec_type) {
            case AVMEDIA_TYPE_VIDEO:
                ret = avcodec_decode_video2(d->avctx, frame, &gotFrame, &d->pkt_temp);
                if (gotFrame) {
//                    if (decoderReorderPts == -1) {
//                        frame->pts = av_frame_get_best_effort_timestamp(frame);
//                    } else if (decoderReorderPts) {
//                        frame->pts = frame->pkt_pts;
//                    } else {
                        frame->pts = frame->pkt_dts;
//                    }
                    
#ifdef DEBUG
                    uint8_t* yuv = new uint8_t[frame->width * frame->height * 3 / 2];
                    int y = frame->width * frame->height;
                    if (frame->width == frame->linesize[0]) {
                        memcpy(yuv, frame->data[0], y);
                        memcpy(yuv + y, frame->data[1], y >> 2);
                        memcpy(yuv + y + (y >> 2), frame->data[2], y >> 2);
                    } else {
                        for (int i = 0; i < frame->height; i++) {
                            memcpy(yuv + i * frame->width, frame->data[0] + i * frame->linesize[0], frame->width);
                        }
                        for (int i = 0; i < frame->height / 2; i++) {
                            memcpy(yuv + y + i * frame->width  / 2, frame->data[1] + i * frame->linesize[1], frame->width / 2);
                        }
                        for (int i = 0; i < frame->height / 2; i++) {
                            memcpy(yuv + y * 5 / 4 + i * frame->width / 2, frame->data[2] + i * frame->linesize[2], frame->width / 2);
                        }
                    }
                    outputStream.write(reinterpret_cast<char*>(yuv), frame->width * frame->height * 3 / 2);
                    delete[] yuv;
#endif
                }
                break;
                
            case AVMEDIA_TYPE_AUDIO:
                ret = avcodec_decode_audio4(d->avctx, frame, &gotFrame, &d->pkt_temp);
                if (gotFrame) {
                    AVRational tb = (AVRational) {1, frame->sample_rate};
                    if (frame->pts != AV_NOPTS_VALUE) {
                        frame->pts = av_rescale_q(frame->pts, d->avctx->time_base, tb);
                    } else if (frame->pkt_pts != AV_NOPTS_VALUE) {
                        frame->pts = av_rescale_q(frame->pkt_pts, av_codec_get_pkt_timebase(d->avctx), tb);
                    } else if (d->next_pts != AV_NOPTS_VALUE) {
                        frame->pts = av_rescale_q(d->next_pts, d->next_pts_tb, tb);
                    }
                    if (frame->pts != AV_NOPTS_VALUE) {
                        d->next_pts = frame->pts + frame->nb_samples;
                        d->next_pts_tb = tb;
                    }
                 }
                break;
                
            case AVMEDIA_TYPE_SUBTITLE:
                ret = avcodec_decode_subtitle2(d->avctx, sub, &gotFrame, &d->pkt_temp);
                break;
            default:
                break;
        }
        if (ret < 0) {
            d->packet_pending = 0;
        } else {
            d->pkt_temp.dts = d->pkt_temp.pts = AV_NOPTS_VALUE;
            if (d->pkt_temp.data) {
                if (d->avctx->codec_type != AVMEDIA_TYPE_AUDIO) {
                    ret = d->pkt_temp.size;
                }
                d->pkt_temp.data += ret;
                d->pkt_temp.size -= ret;
                if (d->pkt_temp.size <= 0) {
                    d->packet_pending = 0;
                }
            } else {
                if (!gotFrame) {
                    d->packet_pending = 0;
                    d->finished = d->pkt_serial;
                }
            }
        }
    } while (!gotFrame && !d->finished);
    return gotFrame;
}

int FFmpegRead::getVideoFrame(VideoState *is, AVFrame *frame) {
    int gotPicture;
    if ((gotPicture = decoderDecodeFrame(&is->viddec, frame, nullptr)) < 0) {
        return gotPicture;
    }
    
    if (gotPicture) {
        double dpts = NAN;
        if (frame->pts != AV_NOPTS_VALUE) {
            dpts = av_q2d(is->video_st->time_base) * frame->pts;
        }
        frame->sample_aspect_ratio = av_guess_sample_aspect_ratio(is->ic, is->video_st, frame);
        is->viddec_width = frame->width;
        is->viddec_height = frame->height;
    }
    return gotPicture;
}

double FFmpegRead::getRotation(AVStream *st) {
    AVDictionaryEntry* rotate = av_dict_get(st->metadata, "rotate", nullptr, 0);
    uint8_t* displaymatrix = av_stream_get_side_data(st, AV_PKT_DATA_DISPLAYMATRIX,
                                                     nullptr);
    double theta = 0;
    if (rotate && *rotate->value && strcmp(rotate->value, "0")) {
        char* tail;
        theta = av_strtod(rotate->value, &tail);
        if (*tail) {
            theta = 0;
        }
    }
    if (displaymatrix && !theta) {
        theta = -av_display_rotation_get((int32_t *) displaymatrix);
    }
    theta -= 360 * floor(theta / 360 + 0.9 / 360);
    if (fabs(theta - 90 * round(theta / 90)) > 2) {
        av_log(nullptr, AV_LOG_WARNING, "Odd rotation angle.\n"
               "If you want to help, upload a sample "
               "of this file to ftp://upload.ffmpeg.org/incoming/ "
               "and contact the ffmpeg-devel mailing list. (ffmpeg-devel@ffmpeg.org)");
    }
    return theta;
}

int FFmpegRead::queuePicture(VideoState* is, AVFrame* src_frame, double pts, double duration, int64_t pos, int serial) {
    Frame* vp;
    if (!(vp = frameQueuePeekWritable(&is->pictq))) {
        return -1;
    }
    vp->sar = src_frame->sample_aspect_ratio;
    vp->pts = pts;
    vp->duration = duration;
    vp->pos = pos;
    vp->serial = serial;
    av_frame_move_ref(vp->frame, src_frame);
    frameQueuePush(&is->pictq);
    return 0;
}

int FFmpegRead::synchronizeAudio(VideoState* is, int nb_samples) {
    int wanted_nb_samples = nb_samples;
    if (getMasterSyncType(is) != AV_SYNC_AUDIO_MASTER) {
        double diff = getClock(&is->audclk) - getMasterClock(is);
        if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD) {
            is->audio_diff_cum = diff + is->audio_diff_avg_coef * is->audio_diff_cum;
            if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB) {
                is->audio_diff_avg_count++;
            } else {
                double avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef);
                if (fabs(avg_diff) >= is->audio_diff_threshold) {
                    wanted_nb_samples = nb_samples + (int) (diff * is->audio_src.freq);
                    int min_nb_samples = ((nb_samples * (100 - SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    int max_nb_samples = ((nb_samples * (100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    wanted_nb_samples = av_clip_c(wanted_nb_samples, min_nb_samples, max_nb_samples);
                }
                av_log(NULL, AV_LOG_TRACE, "diff=%f adiff=%f sample_diff=%d apts=%0.3f %f\n",
                       diff, avg_diff, wanted_nb_samples - nb_samples,
                       is->audio_clock, is->audio_diff_threshold);
            }
        } else {
            is->audio_diff_avg_count = 0;
            is->audio_diff_cum = 0;
        }
    }
    return wanted_nb_samples;
}

int FFmpegRead::audioDecoderFrame(VideoState* is, Frame** audio_frame) {
    if (is->paused) {
        return -1;
    }
    do  {
        if (!(*audio_frame = frameQueuePeekReadable(&is->sampq))) {
            return -1;
        }
    } while ((*audio_frame)->serial != is->audioq.serial);
    
    int resample_data_size;
    Frame* frame = *audio_frame;
    int data_size = av_samples_get_buffer_size(NULL, av_frame_get_channels(frame->frame), frame->frame->nb_samples, (AVSampleFormat) frame->frame->format, 1);
    int64_t dec_channel_layout = (frame->frame->channel_layout && av_frame_get_channels(frame->frame) == av_get_channel_layout_nb_channels(frame->frame->channel_layout) ? frame->frame->channel_layout : av_get_default_channel_layout(av_frame_get_channels(frame->frame)));
    int wanted_nb_samples = synchronizeAudio(is, frame->frame->nb_samples);
    if (frame->frame->format != is->audio_src.fmt ||
        dec_channel_layout != is->audio_src.channel_layout ||
        frame->frame->sample_rate != is->audio_src.freq ||
        (wanted_nb_samples != frame->frame->nb_samples && !is->swr_ctx)) {
        swr_free(&is->swr_ctx);
        is->swr_ctx = swr_alloc_set_opts(NULL, is->audio_tgt.channel_layout, (AVSampleFormat) is->audio_tgt.fmt, is->audio_tgt.freq, dec_channel_layout, (AVSampleFormat) frame->frame->format, frame->frame->sample_rate, 0, NULL);
        if (!is->swr_ctx || swr_init(is->swr_ctx) < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Cannot create sample rate converter for conversion of %d Hz %s %d channels to %d Hz %s %d channels!\n",
                   frame->frame->sample_rate, av_get_sample_fmt_name((AVSampleFormat) frame->frame->format), av_frame_get_channels(frame->frame),
                   is->audio_tgt.freq, av_get_sample_fmt_name(is->audio_tgt.fmt), is->audio_tgt.channels);
            
            swr_free(&is->swr_ctx);
            return -1;
        }
        is->audio_src.channel_layout = dec_channel_layout;
        is->audio_src.channels = av_frame_get_channels(frame->frame);
        is->audio_src.freq = frame->frame->sample_rate;
        is->audio_src.fmt = (AVSampleFormat) frame->frame->format;
    }
    if (is->swr_ctx) {
        const uint8_t** in = (const uint8_t**) frame->frame->extended_data;
        uint8_t** out = &is->audio_buf1;
        int out_count = wanted_nb_samples * is->audio_tgt.freq / frame->frame->sample_rate + 256;
        int out_size = av_samples_get_buffer_size(NULL, is->audio_tgt.channels, out_count, is->audio_tgt.fmt, 0);
        if (out_size < 0) {
            av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size() failed\n");
            return -1;
        }
        if (wanted_nb_samples != frame->frame->nb_samples) {
            if (swr_set_compensation(is->swr_ctx, (wanted_nb_samples - frame->frame->nb_samples) * is->audio_tgt.freq / frame->frame->sample_rate, wanted_nb_samples * is->audio_tgt.freq / frame->frame->sample_rate) < 0) {
                av_log(NULL, AV_LOG_ERROR, "swr_set_compensation() failed\n");
                return -1;
            }
        }
        av_fast_malloc(&is->audio_buf1, &is->audio_buf_size, out_size);
        if (!is->audio_buf1) {
            return AVERROR(ENOMEM);
        }
        int len2 = swr_convert(is->swr_ctx, out, out_count, in, frame->frame->nb_samples);
        if (len2 < 0) {
            av_log(NULL, AV_LOG_ERROR, "swr_convert() failed\n");
            return -1;
        }
        if (len2 == out_count) {
            av_log(NULL, AV_LOG_WARNING, "audio buffer is probably too small.\n");
            if (swr_init(is->swr_ctx) < 0) {
                swr_free(&is->swr_ctx);
            }
        }
        is->audio_buf = is->audio_buf1;
        resample_data_size = len2 * is->audio_tgt.channels * av_get_bytes_per_sample(is->audio_tgt.fmt);
    } else {
        is->audio_buf = frame->frame->data[0];
        resample_data_size = data_size;
    }
    int audio_clock = is->audio_clock;
    if (!isnan(frame->pts)) {
        is->audio_clock = frame->pts + (double) frame->frame->nb_samples / frame->frame->sample_rate;
    } else {
        is->audio_clock = NAN;
    }
    is->audio_clock_serial = frame->serial;
    return resample_data_size;
}

void* FFmpegRead::audioThread(void *arg) {
    VideoState* is = reinterpret_cast<VideoState*>(arg);
    AVFrame* frame = av_frame_alloc();
    if (frame == nullptr) {
        return nullptr;
    }
    int ret = 0;
    int gotFrame;
    while (1) {
        ret = is->object->decoderDecodeFrame(&is->auddec, frame, nullptr);
        if (ret < 0) {
            return 0;
        }
        if (!ret) {
            continue;
        }
    }
    return nullptr;
}

int FFmpegRead::streamComponentOpen(VideoState *is, int streamIndex, FFmpegRead* read) {
    int ret;
    AVFormatContext* ic = is->ic;
    AVCodecContext* avctx;
    AVCodec* codec;
    AVDictionary* opt = nullptr;
    int sampleRate, nbChannels;
    int64_t channelLayout;
    int streamLowres = is->object->lowres;
    if (streamIndex < 0 || streamIndex >= ic->nb_streams) {
        return -1;
    }
    avctx = avcodec_alloc_context3(nullptr);
    if (avctx == nullptr) {
        return AVERROR(ENOMEM);
    }
    ret = avcodec_parameters_to_context(avctx, ic->streams[streamIndex]->codecpar);
    if (ret < 0) {
        av_err2str(ret);
        avcodec_free_context(&avctx);
        // TODO return
        return -1;
    }
    av_codec_set_pkt_timebase(avctx, ic->streams[streamIndex]->time_base);
    codec = avcodec_find_decoder(avctx->codec_id);
    if (codec == nullptr) {
        av_log(nullptr, AV_LOG_WARNING, "No Codec could be found with id %d\n", (int) (avctx->codec_id));
        avcodec_free_context(&avctx);
        return AVERROR(ENOMEM);
    }
    avctx->codec_id = codec->id;
    if (streamLowres > av_codec_get_max_lowres(codec)) {
        av_log(avctx, AV_LOG_WARNING, "The maximum value for lowres supported by the decoder is %d", av_codec_get_max_lowres(codec));
        streamLowres = av_codec_get_max_lowres(codec);
    }
    av_codec_set_lowres(avctx, streamLowres);
#if FF_API_EMU_EDGE
    if (streamLowres) avctx->flags |= CODEC_FLAG_EMU_EDGE;
#endif
    if (is->object->fast) {
        avctx->flags |= AV_CODEC_FLAG2_FAST;
    }
#if FF_API_EMU_EDGE
    if (codec->capabilities & AV_CODEC_CAP_DR1) {
        avctx->flags |= CODEC_FLAG_EMU_EDGE;
    }
#endif
    av_dict_set(&opt, "threads", "auto", 0);
    if (streamLowres) {
        char streamLowresStr[1024];
        sprintf(streamLowresStr, "%d", streamLowres);
        av_dict_set(&opt, "lowers", streamLowresStr, 0);
    }
    if (avctx->codec_type == AVMEDIA_TYPE_VIDEO || avctx->codec_type == AVMEDIA_TYPE_AUDIO) {
        av_dict_set(&opt, "refcounted_frames", "1", 0);
    }
    if ((ret = avcodec_open2(avctx, codec, &opt)) < 0) {
        avcodec_free_context(&avctx);
        av_err2str(ret);
        return ret;
    }
    is->eof = 0;
    ic->streams[streamIndex]->discard = AVDISCARD_DEFAULT;
    
    switch (avctx->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            // audio open
            
            is->audio_buf_size = 0;
            is->audio_buf_index = 0;
            
            is->audio_diff_avg_coef = exp(log(0.01) / AUDIO_DIFF_AVG_NB);
            is->audio_stream = streamIndex;
            is->audio_st = ic->streams[streamIndex];
            decoderInit(&is->auddec, avctx, &is->audioq, is->continue_read_thread);
            if ((is->ic->iformat->flags & (AVFMT_NOBINSEARCH | AVFMT_NOGENSEARCH | AVFMT_NO_BYTE_SEEK)) && !is->ic->iformat->read_seek) {
                is->auddec.start_pts = is->audio_st->start_time;
                is->auddec.start_pts_tb = is->audio_st->time_base;
            }
            ret = decoderStart(&is->auddec, audioThread, is, read);
            if (ret < 0) {
                avcodec_free_context(&avctx);
                return ret;
            }
            break;
            
        case AVMEDIA_TYPE_VIDEO:
            is->video_stream = streamIndex;
            is->video_st = ic->streams[streamIndex];
            is->viddec_width = avctx->width;
            is->viddec_height = avctx->height;
            decoderInit(&is->viddec, avctx, &is->videoq, is->continue_read_thread);
            ret = decoderStart(&is->viddec, videoThread, is, read);
            if (ret < 0) {
                avcodec_free_context(&avctx);
                return ret;
            }
            is->queue_attachments_req = 1;
            break;
        default:
            break;
    }
    return 0;
}

void FFmpegRead::readThreadFailed(VideoState *is, AVFormatContext *ic, pthread_mutex_t waitMutex) {
    if (ic && !is->ic) {
        avformat_close_input(&ic);
    }
    // TODO 读取出错
    pthread_mutex_destroy(&waitMutex);
}

void* FFmpegRead::readThread(void *arg) {
    VideoState* is = reinterpret_cast<VideoState*>(arg);
    AVFormatContext* ic = nullptr;
    int ret;
    int stIndex[AVMEDIA_TYPE_NB];
    AVPacket pkt1, *pkt = &pkt1;
    int64_t streamStartTime;
    pthread_mutex_t waitMutex;
    ret = pthread_mutex_init(&waitMutex, nullptr);
    if (ret != 0) {
        readThreadFailed(is, ic, waitMutex);
        return nullptr;
    }
    memset(stIndex, -1, sizeof(stIndex));
    is->last_video_stream = is->video_stream = -1;
    is->last_audio_stream = is->audio_stream = -1;
    is->last_subtitle_stream = is->subtitle_stream = -1;
    is->eof = 0;
    
    ic = avformat_alloc_context();
    if (ic == nullptr) {
        av_log(nullptr, AV_LOG_FATAL, "Can't allocate context.\n");
        readThreadFailed(is, ic, waitMutex);
        return nullptr;
    }
    ic->interrupt_callback.callback = decodeInterruptCb;
    ic->interrupt_callback.opaque = is;
    ret = avformat_open_input(&ic, is->filename, nullptr, nullptr);
    if (ret < 0) {
        readThreadFailed(is, ic, waitMutex);
        av_err2str(ret);
        return nullptr;
    }
    is->ic = ic;
    av_format_inject_global_side_data(ic);
    ret = avformat_find_stream_info(ic, nullptr);
    if (ret < 0) {
        readThreadFailed(is, ic, waitMutex);
        av_err2str(ret);
        return nullptr;
    }
    if (ic->pb) {
        ic->pb->eof_reached = 0;
    }
    is->max_frame_duration = (ic->iformat->flags & AVFMT_TS_DISCONT) ? 10.0 : 3600.0;
    if (is->object->startTime != AV_NOPTS_VALUE) {
        int64_t timestamp = is->object->startTime;
        // add the stream time
        if (ic->start_time != AV_NOPTS_VALUE) {
            timestamp += ic->start_time;
        }
        ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
        if (ret < 0) {
            av_err2str(ret);
            av_log(nullptr, AV_LOG_WARNING, "%s: could not seek to position %0.3f\n", is->filename, (double) timestamp / AV_TIME_BASE);
        }
    }
    av_dump_format(ic, 0, is->filename, 0);
    for (int i = 0; i < ic->nb_streams; i++) {
        AVStream* st = ic->streams[i];
        enum AVMediaType type = st->codecpar->codec_type;
        st->discard = AVDISCARD_ALL;
        if (is->object->wanted_stream_spec[type] && stIndex[i] == -1) {
            if (avformat_match_stream_specifier(ic, st, is->object->wanted_stream_spec[type]) > 0) {
                stIndex[type] = i;
            }
        }
    }
    for (int i = 0; i < AVMEDIA_TYPE_NB; i++) {
        if (is->object->wanted_stream_spec[i] && stIndex[i] == -1) {
            av_log(nullptr, AV_LOG_ERROR, "Stream specifier %s does not match any %s stream", is->object->wanted_stream_spec[i], av_get_media_type_string((AVMediaType)i));
            stIndex[i] = INT_MAX;
        }
    }
    stIndex[AVMEDIA_TYPE_VIDEO] = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO, stIndex[AVMEDIA_TYPE_VIDEO], -1, nullptr, 0);
    
    stIndex[AVMEDIA_TYPE_AUDIO] = av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO, stIndex[AVMEDIA_TYPE_AUDIO], -1, nullptr, 0);
    
    stIndex[AVMEDIA_TYPE_SUBTITLE] = av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE, stIndex[AVMEDIA_TYPE_SUBTITLE], stIndex[AVMEDIA_TYPE_AUDIO] > 0 ? stIndex[AVMEDIA_TYPE_AUDIO] : stIndex[AVMEDIA_TYPE_VIDEO], nullptr, 0);
    
    if (stIndex[AVMEDIA_TYPE_VIDEO] >= 0) {
        AVStream* st = ic->streams[stIndex[AVMEDIA_TYPE_VIDEO]];
        AVCodecParameters* codecpar = st->codecpar;
        AVRational sar = av_guess_sample_aspect_ratio(ic, st, nullptr);
        if (codecpar->width) {
            // set default window size
        }
    }
    // open the streams
    if (stIndex[AVMEDIA_TYPE_VIDEO] >= 0) {
        ret = streamComponentOpen(is, stIndex[AVMEDIA_TYPE_VIDEO], is->object);
        if (ret < 0) {
            return nullptr;
        }
    }
    ret = -1;
    if (stIndex[AVMEDIA_TYPE_AUDIO] >= 0) {
        ret = streamComponentOpen(is, stIndex[AVMEDIA_TYPE_AUDIO], is->object);
        if (ret < 0) {
            return nullptr;
        }
    }
    if (is->show_mode == VideoState::SHOW_MODE_NONE) {
        is->show_mode = ret >= 0 ? VideoState::SHOW_MODE_VIDEO : VideoState::SHOW_MODE_RDFT;
    }
    if (stIndex[AVMEDIA_TYPE_SUBTITLE] >= 0) {
        streamComponentOpen(is, stIndex[AVMEDIA_TYPE_SUBTITLE], is->object);
    }
    if (is->video_stream < 0 && is->audio_stream < 0) {
        readThreadFailed(is, ic, waitMutex);
        return 0;
    }
    
    for (;;) {
        if (is->abort_request) {
            break;
        }
        if (is->paused != is->last_paused) {
            is->last_paused = is->paused;
            if (is->paused) {
                is->read_pause_return = av_read_pause(ic);
            } else {
                av_read_play(ic);
            }
        }
#if CONFIG_RTSP_DEMUXER || CONFIG_MMSH_PROTOCOL
        if (is->paused && !(strcmp(ic->iformat->name, "rtsp") || (ic->pb && !strncmp(is->filename, "mmsh:", 5)))) {
            /* wait 10 ms to avoid trying to get another packet */
            SDL_Delay(10);
            continue;
        }
#endif
        if (is->seek_req) {
            int64_t seek_target = is->seek_pos;
            int64_t seek_min = is->seek_rel > 0 ? seek_target - is->seek_rel + 2 : INT64_MIN;
            int64_t seek_max = is->seek_rel < 0 ? seek_target - is->seek_rel - 2 : INT64_MAX;
            
            // FIXME the +-2 is due to rounding being not done in the correct direction in generation
            //      of the seek_pos/seek_rel variables
            ret = avformat_seek_file(ic, -1, seek_min, seek_target, seek_max, is->seek_flags);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR, "%s: error while seeking\n", is->filename);
            } else {
                if (is->audio_stream >= 0) {
                    is->object->packetQueueFlush(&is->audioq);
                    is->object->packetQueuePut(&is->audioq, &is->object->flushPacket);
                }
                if (is->video_stream >= 0) {
                    is->object->packetQueueFlush(&is->videoq);
                    is->object->packetQueuePut(&is->videoq, &is->object->flushPacket);
                }
                if (is->subtitle_stream >= 0) {
                    is->object->packetQueueFlush(&is->subtitleq);
                    is->object->packetQueuePut(&is->subtitleq, &is->object->flushPacket);
                }
                if (is->seek_flags & AVSEEK_FLAG_BYTE) {
                    is->object->setClock(&is->extclk, NAN, 0);
                } else {
                    is->object->setClock(&is->extclk, seek_target / (double) AV_TIME_BASE, 0);
                }
            }
            is->seek_req = 0;
            is->queue_attachments_req = 1;
            is->eof = 0;
            if (is->paused) {
                is->object->stepToNextFrame(is);
            }
        }
        if (is->queue_attachments_req) {
            if (is->video_st && is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC) {
                AVPacket copy;
                if ((ret = av_copy_packet(&copy, &is->video_st->attached_pic)) < 0) {
                    is->object->readThreadFailed(is, ic, waitMutex);
                }
                is->object->packetQueuePut(&is->videoq, &copy);
                is->object->packetQueuePutNullPacket(&is->videoq, is->video_stream);
            }
            is->queue_attachments_req = 0;
        }
        
        /* if the queue are full, no need to read more */
        if (is->object->infinite_buffer < 1 && (is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE ||
                                    ((is->audioq.nb_packets > MIN_FRAMES || is->audio_stream < 0 || is->audioq.abort_request) &&
                                     (is->videoq.nb_packets > MIN_FRAMES || is->video_stream < 0 || is->videoq.abort_request ||
                                      (is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)) &&
                                     (is->subtitleq.nb_packets > MIN_FRAMES || is->subtitle_stream < 0 || is->subtitleq.abort_request)))) {
                                        /* wait 10 ms */
                                        // TODO wait time out
                                        pthread_mutex_lock(&waitMutex);
                                        struct timeval delta;
                                        struct timespec abstime;
                                        uint32_t ms = 10;
                                        gettimeofday(&delta, NULL);
                                        abstime.tv_sec = delta.tv_sec + (ms/1000);
                                        abstime.tv_nsec = (delta.tv_usec + (ms%1000) * 1000) * 1000;
                                        if ( abstime.tv_nsec > 1000000000 ) {
                                            abstime.tv_sec += 1;
                                            abstime.tv_nsec -= 1000000000;
                                        }
                                        
                                        pthread_cond_timedwait(&is->continue_read_thread, &waitMutex, &abstime);
                                        pthread_mutex_unlock(&waitMutex);
                                        continue;
                                    }
        if (!is->paused &&
            (!is->audio_st || (is->auddec.finished == is->audioq.serial && is->object->frameQueueNbRemaining(&is->sampq) == 0)) &&
            (!is->video_st || (is->viddec.finished == is->videoq.serial && is->object->frameQueueNbRemaining(&is->pictq) == 0))) {
            if (is->object->loop != 1 && (!is->object->loop || --is->object->loop)) {
                is->object->streamSeek(is, is->object->startTime != AV_NOPTS_VALUE ? is->object->startTime : 0, 0, 0);
            } else if (is->object->autoexit) {
                ret = AVERROR_EOF;
                is->object->readThreadFailed(is, ic, waitMutex);
                return 0;
            }
        }
        ret = av_read_frame(ic, pkt);
        if (ret < 0) {
            if ((ret == AVERROR_EOF || avio_feof(ic->pb)) && !is->eof) {
                if (is->video_stream >= 0) {
                    is->object->packetQueuePutNullPacket(&is->videoq, is->video_stream);
                }
                if (is->audio_stream >= 0) {
                    is->object->packetQueuePutNullPacket(&is->audioq, is->audio_stream);
                }
                if (is->subtitle_stream >= 0) {
                    is->object->packetQueuePutNullPacket(&is->subtitleq, is->subtitle_stream);
                }
                is->eof = 1;
            }
            if (ic->pb && ic->pb->error) {
                break;
            }
            // TODO wait time out 10ms
            pthread_mutex_lock(&waitMutex);
            struct timeval delta;
            struct timespec abstime;
            uint32_t ms = 10;
            gettimeofday(&delta, NULL);
            abstime.tv_sec = delta.tv_sec + (ms/1000);
            abstime.tv_nsec = (delta.tv_usec + (ms%1000) * 1000) * 1000;
            if ( abstime.tv_nsec > 1000000000 ) {
                abstime.tv_sec += 1;
                abstime.tv_nsec -= 1000000000;
            }
            
            pthread_cond_timedwait(&is->continue_read_thread, &waitMutex, &abstime);
            pthread_mutex_unlock(&waitMutex);
            continue;
        } else {
            is->eof = 0;
        }
        /* check if packet is in play range specified by user, then queue, otherwise discard */
        streamStartTime = ic->streams[pkt->stream_index]->start_time;
        int64_t pkt_ts = pkt->pts == AV_NOPTS_VALUE ? pkt->dts : pkt->pts;
        int pkt_in_play_range = is->object->duration == AV_NOPTS_VALUE || (pkt_ts - (streamStartTime != AV_NOPTS_VALUE ? streamStartTime : 0)) *
        av_q2d(ic->streams[pkt->stream_index]->time_base) -
        (double)(is->object->startTime != AV_NOPTS_VALUE ? is->object->startTime : 0) / 1000000 <= ((double) is->object->duration / 1000000);
        if (pkt->stream_index == is->audio_stream && pkt_in_play_range) {
            is->object->packetQueuePut(&is->audioq, pkt);
        } else if (pkt->stream_index == is->video_stream && pkt_in_play_range && !(is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)) {
            is->object->packetQueuePut(&is->videoq, pkt);
        } else if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range) {
            is->object->packetQueuePut(&is->subtitleq, pkt);
        } else {
            av_packet_unref(pkt);
        }
    }
    return nullptr;
}

VideoState* FFmpegRead::streamOpen(const char *filename) {
    VideoState* is = new VideoState();
    if (is == nullptr) {
        return nullptr;
    }
    is->object = this;
    is->filename = av_strdup(filename);
    if (is->filename == nullptr) {
        streamClose(is);
        return nullptr;
    }
    if (frameQueueInit(&is->pictq, &is->videoq, VIDEO_PICTURE_QUEUE_SIZE, 1) < 0) {
        streamClose(is);
        return nullptr;
    }
    if (frameQueueInit(&is->subpq, &is->subtitleq, SUBPICTURE_QUEUE_SIZE, 0) < 0) {
        streamClose(is);
        return nullptr;
    }
    if (frameQueueInit(&is->sampq, &is->audioq, SAMPLE_QUEUE_SIZE, 1) < 0) {
        streamClose(is);
        return nullptr;
    }
    if (packetQueueInit(&is->videoq) < 0 ||
        packetQueueInit(&is->audioq) < 0 ||
        packetQueueInit(&is->subtitleq) < 0) {
        streamClose(is);
        return nullptr;
    }
    int result = pthread_cond_init(&is->continue_read_thread, nullptr);
    if (result != 0) {
        streamClose(is);
        return nullptr;
    }
    initClock(&is->vidclk, &is->videoq.serial);
    initClock(&is->audclk, &is->audioq.serial);
    initClock(&is->extclk, &is->extclk.serial);
    is->audio_clock_serial = -1;
    is->audio_volume = 128;
    is->muted = 0;
    is->av_sync_type = av_sync_type;
    result = pthread_create(&is->read_tid, nullptr, readThread, is);
    if (result != 0) {
        streamClose(is);
        return nullptr;
    }
    pthread_join(is->read_tid, nullptr);
    return is;
}

void FFmpegRead::decoderRelease(FFmpegRead *read, Decoder *d) {
    av_packet_unref(&d->pkt);
    avcodec_free_context(&d->avctx);
}

void FFmpegRead::decoderAbort(FFmpegRead* read, Decoder *d, FrameQueue *fq) {
    read->packetQueueAbort(d->queue);
    read->frameQueueSignal(fq);
    pthread_join(d->decoder_tid, nullptr);
    d->decoder_tid = nullptr;
    read->packetQueueFlush(d->queue);
}

void FFmpegRead::streamComponentClose(VideoState *is, int streamIndex) {
    AVFormatContext* ic = is->ic;
    AVCodecContext* avctx;
    if (streamIndex < 0 || streamIndex >= ic->nb_streams) {
        return;
    }
    avctx = ic->streams[streamIndex]->codec;
    switch (avctx->codec_type) {
        case AVMEDIA_TYPE_VIDEO:
            decoderAbort(is->object, &is->viddec, &is->pictq);
            decoderRelease(is->object, &is->viddec);
            break;
            
        case AVMEDIA_TYPE_AUDIO:
            decoderAbort(is->object, &is->auddec, &is->sampq);
            decoderRelease(is->object, &is->auddec);
            av_freep(&is->audio_buf1);
            is->audio_buf1_size = 0;
            is->audio_buf = nullptr;
            break;
        default:
            break;
    }
    ic->streams[streamIndex]->discard = AVDISCARD_ALL;
    avcodec_close(avctx);
    switch (avctx->codec_type) {
        case AVMEDIA_TYPE_VIDEO:
            is->audio_st = nullptr;
            is->audio_stream = -1;
            break;
            
        case AVMEDIA_TYPE_AUDIO:
            is->video_st = nullptr;
            is->video_stream = -1;
            break;
            
        case AVMEDIA_TYPE_SUBTITLE:
            is->subtitle_st = nullptr;
            is->subtitle_stream = -1;
            break;
        default:
            break;
    }
}

void FFmpegRead::streamClose(VideoState *is) {
    is->abort_request = 1;
    pthread_join(is->read_tid, nullptr);
    if (is->audio_stream >= 0) {
        streamComponentClose(is, is->audio_stream);
    }
    if (is->video_stream >= 0) {
        streamComponentClose(is, is->video_stream);
    }
    if (is->subtitle_stream >= 0) {
        streamComponentClose(is, is->subtitle_stream);
    }
    avformat_close_input(&is->ic);
    packetQueueDestroy(&is->videoq);
    packetQueueDestroy(&is->audioq);
    packetQueueDestroy(&is->subtitleq);
    pthread_cond_destroy(&is->continue_read_thread);
    av_freep(is->filename);
    delete is;
}

int FFmpegRead::init(const char *filename) {
#ifdef DEBUG
    outputStream.open("/Users/wlanjie/Desktop/ffmpeg.h264", std::ios_base::binary | std::ios_base::out);
#endif
    av_init_packet(&flushPacket);
    flushPacket.data = (uint8_t*) &flushPacket;
    flushPacket.size = 0;
    VideoState* vs = streamOpen(filename);
    return 0;
}
