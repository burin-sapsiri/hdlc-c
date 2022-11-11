#pragma once
#ifndef _HDLC_H
#define _HDLC_H

#include <stdint.h>

typedef enum {
    HDLC_PARSE_RESULT_IDLE = 0,
    HDLC_PARSE_RESULT_PENDING,
    HDLC_PARSE_RESULT_COMPLETE,
}hdlc_parse_result_t;

typedef uint8_t hdlc_state_t;

typedef struct {
    uint8_t                 *buf;
    uint32_t                buf_len;
    uint32_t                buf_position;

    hdlc_state_t            state;
}hdlc_t;

#ifdef __cplusplus
extern "C" {
#endif

void hdlc_init(hdlc_t* p_hndl, uint8_t* buf, uint32_t len);
hdlc_parse_result_t hdlc_input_byte(hdlc_t* p_hndl, uint8_t data);
uint32_t hdlc_get_input_frame_len(hdlc_t* p_hndl);
uint32_t hdlc_get_input_frame(hdlc_t* p_hndl, uint8_t* data, uint32_t max_len);
const uint8_t *hdlc_get_input_frame_ptr(hdlc_t* p_hndl);
uint32_t hdlc_write_output_frame(hdlc_t* p_hndl,
                                 const uint8_t* data,
                                 uint32_t len,
                                 uint8_t* output,
                                 uint32_t output_max_len);
void hdlc_clear(hdlc_t* p_hndl);

#ifdef __cplusplus
}
#endif

#endif /* _HDLC_H */
