//
//  ffmpeg.c
//  ffplay
//
//  Created by wlanjie on 16/7/30.
//  Copyright © 2016年 com.wlanjie.ffplay. All rights reserved.
//

#include "ffmpeg.h"


InputFile *input_file = NULL;
InputStream **input_streams = NULL;
int nb_input_streams = 0;
OutputFile *output_file = NULL;
OutputStream **output_streams = NULL;
int nb_output_streams = 0;
int64_t start_time = 0;
int framecnt = 0;

void *grow_array(void *array, int elem_size, int *size, int new_size) {
    if (new_size > INT_MAX / elem_size) {
        av_log(NULL, AV_LOG_ERROR, "Array to big.\n");
        return NULL;
    }
    if (*size < new_size) {
        uint8_t *tmp = av_realloc_array(array, (size_t) elem_size, (size_t) new_size);
        memset(tmp + *size * elem_size, 0, (size_t) ((new_size - *size) * elem_size));
        *size = new_size;
        return tmp;
    }
    return array;
}

double get_rotation(AVStream *st) {
    AVDictionaryEntry *rotate = av_dict_get(st->metadata, "rotate", NULL, 0);
    uint8_t *displaymatrix = av_stream_get_side_data(st, AV_PKT_DATA_DISPLAYMATRIX, NULL);
    double theta = 0;
    if (rotate && *rotate->value && strcmp(rotate->value, "0")) {
        char *tail;
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
        av_log(NULL, AV_LOG_WARNING, "Odd rotation angle.\n"
               "If you want to help, upload a sample "
               "of this file to ftp://upload.ffmpeg.org/incoming/ "
               "and contact the ffmpeg-devel mailing list. (ffmpeg-devel@ffmpeg.org)");
    }
    return theta;
}

AVPacket init_packet() {
    AVPacket packet;
    av_init_packet(&packet);
    packet.size = 0;
    packet.data = NULL;
    return packet;
}

int decode_video(AVPacket packet, int *got_output) {
    int ret = 0;
    InputStream *ist = input_streams[packet.stream_index];
    AVFrame *frame = av_frame_alloc();
    ret = avcodec_decode_video2(ist->dec_ctx, frame, got_output, &packet);
    if (ret < 0) {
        av_frame_free(&frame);
        av_log(NULL, AV_LOG_ERROR, "avcodec_decode_video2 error: %s\n", av_err2str(ret));
        return ret;
    }
    if (*got_output) {
    }
    frame->pts = av_frame_get_best_effort_timestamp(frame);
    int err = av_buffersrc_add_frame_flags(ist->filter, frame, AV_BUFFERSRC_FLAG_PUSH);
    if (err == AVERROR_EOF) {
        err = 0;
    }
    av_frame_free(&frame);
    return ret;
}

int decodec_audio(AVPacket packet, int *got_output) {
    int ret = 0;
    InputStream *ist = input_streams[packet.stream_index];
    AVFrame *frame = av_frame_alloc();
    ret = avcodec_decode_audio4(ist->dec_ctx, frame, got_output, &packet);
    if (ret < 0) {
        av_frame_free(&frame);
        av_log(NULL, AV_LOG_ERROR, "avcodec_decode_audio4 error: %s\n", av_err2str(ret));
        return ret;
    }
    if (*got_output) {
        
    }
    frame->pts = frame->pkt_pts;
    int64_t last = AV_NOPTS_VALUE;
    if (frame->pts != AV_NOPTS_VALUE) {
        frame->pts = av_rescale_delta(ist->st->time_base, frame->pts,
                                      (AVRational) { 1, ist->dec_ctx->sample_rate }, frame->nb_samples,
                                      &last, (AVRational) { 1, ist->dec_ctx->sample_rate });
    }
    int err = av_buffersrc_add_frame_flags(ist->filter, frame, AV_BUFFERSRC_FLAG_PUSH);
    if (err == AVERROR_EOF) {
        err = 0;
    }
    frame->pts = AV_NOPTS_VALUE;
    av_frame_free(&frame);
    return ret;
}

int encode_video(AVPacket packet, int *got_output) {
    int ret = 0;
    OutputStream *ost = output_streams[packet.stream_index];
    AVFrame *frame = av_frame_alloc();
    ret = av_buffersink_get_frame_flags(ost->filter, frame, AV_BUFFERSINK_FLAG_NO_REQUEST);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        av_frame_free(&frame);
        return 0;
    }
    frame->pts = av_rescale_q(frame->pts, ost->filter->inputs[0]->time_base, ost->enc_ctx->time_base) -
                av_rescale_q(0, AV_TIME_BASE_Q, ost->enc_ctx->time_base);
//    printf("pts = %lld frame_pts = %lld frame_dts = %lld\n", frame->pts, frame->pkt_pts, frame->pkt_dts);
    frame->pict_type = 0;
    frame->quality = ost->enc_ctx->global_quality;
    AVPacket avpkt;
    avpkt.data = NULL;
    avpkt.size = 0;
    av_init_packet(&avpkt);
    ret = avcodec_encode_video2(ost->enc_ctx, &avpkt, frame, got_output);
    if (ret < 0) {
        av_frame_free(&frame);
        av_packet_unref(&avpkt);
        return ret;
    }
    if (!(*got_output)) {
        return 0;
    }
    
    av_packet_rescale_ts(&avpkt, ost->enc_ctx->time_base, ost->st->time_base);
    avpkt.stream_index = packet.stream_index;
    ret = av_interleaved_write_frame(output_file->oc, &avpkt);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "av_interleaved_write_frame error: %s\n", av_err2str(ret));
    }
    av_packet_unref(&avpkt);
    av_frame_free(&frame);
    return ret;
}

int encode_audio(AVPacket packet, int *got_output) {
    int ret = 0;
    OutputStream *ost = output_streams[packet.stream_index];
    AVFrame *frame = av_frame_alloc();
    ret = av_buffersink_get_frame_flags(ost->filter, frame, AV_BUFFERSINK_FLAG_NO_REQUEST);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        av_frame_free(&frame);
        return 0;
    }
    frame->pts = av_rescale_q(frame->pts, ost->filter->inputs[0]->time_base, ost->enc_ctx->time_base) -
    av_rescale_q(0, AV_TIME_BASE_Q, ost->enc_ctx->time_base);
//    frame->pict_type = AV_PICTURE_TYPE_NONE;
    AVPacket avpkt;
    avpkt.data = NULL;
    avpkt.size = 0;
    av_init_packet(&avpkt);
    ret = avcodec_encode_audio2(ost->enc_ctx, &avpkt, frame, got_output);
    if (ret < 0) {
        av_frame_free(&frame);
        av_packet_unref(&avpkt);
        return ret;
    }
    if (!(*got_output)) {
        return 0;
    }
    avpkt.stream_index = packet.stream_index;
    av_packet_rescale_ts(&avpkt, ost->st->codec->time_base, ost->st->time_base);
    ret = av_interleaved_write_frame(output_file->oc, &avpkt);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "av_interleaved_write_frame error: %s\n", av_err2str(ret));
    }
    av_packet_unref(&avpkt);
    av_frame_free(&frame);
    return ret;
}

int transcode() {
    int ret = 0;
    AVPacket packet;
    packet.data = NULL;
    packet.size = 0;
    
    while (av_read_frame(input_file->ic, &packet) >= 0) {
        int stream_index = packet.stream_index;
        int got_output;
        av_packet_rescale_ts(&packet, input_file->ic->streams[stream_index]->time_base,
                             input_file->ic->streams[stream_index]->codec->time_base);
        if (input_file->ic->streams[stream_index]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
            ret = decode_video(packet, &got_output);
            packet.data += ret;
            packet.size = 0;
        } else if (input_file->ic->streams[stream_index]->codec->codec_type == AVMEDIA_TYPE_AUDIO) {
            ret = decodec_audio(packet, &got_output);
            packet.data += ret;
            packet.size -= ret;
        }
        if (got_output) {
            if (output_file->oc->streams[stream_index]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
                ret = encode_video(packet, &got_output);
            } else if (output_file->oc->streams[stream_index]->codec->codec_type == AVMEDIA_TYPE_AUDIO) {
                ret = encode_audio(packet, &got_output);
            }
        }
    }
    ret = av_write_trailer(output_file->oc);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "av_write_trailer error: %s\n", av_err2str(ret));
        return ret;
    }
    return ret;
}
