#include "hdlc.h"

#define FRAME_BOUNDARY_OCTET                0x7E
#define CONTROL_ESCAPE_OCTET                0x7D

#define HDLC_STATE_IDLE                     0U
#define HDLC_STATE_RECEIVING                1U
#define HDLC_STATE_RECEIVING_STUFFED        2U

#define FLAG_LEN                            2U
#define FCS_LEN                             2U
#define CONTROL_ESCAPE_INVERT               0x20


static const uint16_t FCS_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


#define FCS_INIT_VALUE                  0xffff  /* Initial FCS value */
#define FCS_FINAL_XOR                   0xffff

static uint16_t calculate_fcs(uint16_t fcs, const uint8_t* data, uint32_t len);
static uint16_t calculate_stuffed_byte_count(const uint8_t* data, uint32_t len);

static hdlc_parse_result_t handle_state_idle(hdlc_t* p_hndl, uint8_t data);
static hdlc_parse_result_t handle_state_receiving(hdlc_t* p_hndl, uint8_t data);
static hdlc_parse_result_t handle_state_receiving_stuffed(hdlc_t* p_hndl, uint8_t data);


void hdlc_init(hdlc_t* p_hndl, uint8_t* buf, uint32_t len)
{
    p_hndl->buf = buf;
    p_hndl->buf_len = len;
    p_hndl->state = HDLC_STATE_IDLE;
}

hdlc_parse_result_t hdlc_input_byte(hdlc_t* p_hndl, uint8_t data)
{
    switch (p_hndl->state) {
    case HDLC_STATE_IDLE:
        return handle_state_idle(p_hndl, data);
    case HDLC_STATE_RECEIVING:
        return handle_state_receiving(p_hndl, data);
    case HDLC_STATE_RECEIVING_STUFFED:
        return handle_state_receiving_stuffed(p_hndl, data);
    }

    return HDLC_PARSE_RESULT_IDLE;
}

uint32_t hdlc_get_input_frame_len(hdlc_t* p_hndl)
{
    return p_hndl->buf_position - FCS_LEN;
}

uint32_t hdlc_get_input_frame(hdlc_t* p_hndl, uint8_t* data, uint32_t max_len)
{
    uint32_t len = p_hndl->buf_position - FCS_LEN;

    if (len > max_len) {
        len = max_len;
    }

    memcpy(data, p_hndl->buf, len);

    return len;
}

const uint8_t* hdlc_get_input_frame_ptr(hdlc_t *p_hndl)
{
    return p_hndl->buf;
}

uint32_t hdlc_write_output_frame(hdlc_t* p_hndl,
    const uint8_t* data,
    uint32_t len,
    uint8_t* output,
    uint32_t output_max_len)
{
    uint32_t i = 0;
    uint32_t output_pos = 0;

    uint16_t fcs = 0;
    uint8_t* fcs_ptr = (uint8_t*)&fcs;
    uint32_t stuffed_byte_count = 0;
    uint32_t total_output_len = 0;

    fcs = calculate_fcs(FCS_INIT_VALUE, data, len);

    stuffed_byte_count += calculate_stuffed_byte_count(data, len);
    stuffed_byte_count += calculate_stuffed_byte_count(fcs_ptr, FCS_LEN);

    total_output_len = FLAG_LEN + len + stuffed_byte_count + FCS_LEN;

    if (total_output_len > output_max_len) {
        return 0;
    }

    /* Fill start flag. */
    output[output_pos++] = FRAME_BOUNDARY_OCTET;

    /* Fill data with transparency. */
    for (i = 0; i < len; i++) {
        if ((data[i] == FRAME_BOUNDARY_OCTET) || (data[i] == CONTROL_ESCAPE_OCTET)) {
            output[output_pos++] = CONTROL_ESCAPE_OCTET;
            output[output_pos++] = data[i] ^ CONTROL_ESCAPE_INVERT;
        }
        else {
            output[output_pos++] = data[i];
        }
    }

    /* Fill fcs with transparency. */
    for (i = 0; i < FCS_LEN; i++) {
        if ((fcs_ptr[i] == FRAME_BOUNDARY_OCTET) || (fcs_ptr[i] == CONTROL_ESCAPE_OCTET)) {
            output[output_pos++] = CONTROL_ESCAPE_OCTET;
            output[output_pos++] = fcs_ptr[i] ^ CONTROL_ESCAPE_INVERT;
        }
        else {
            output[output_pos++] = fcs_ptr[i];
        }
    }

    /* Fill end flag. */
    output[output_pos++] = FRAME_BOUNDARY_OCTET;

    return output_pos;
}

void hdlc_clear(hdlc_t* p_hndl)
{
    p_hndl->state = HDLC_STATE_IDLE;
    p_hndl->buf_position = 0;
}




static uint16_t calculate_fcs(uint16_t fcs, const uint8_t* data, uint32_t len)
{
    while (len--)
        fcs = (fcs >> 8) ^ FCS_TABLE[(fcs ^ *data++) & 0xff];

    fcs ^= FCS_FINAL_XOR;

    return (fcs);
}

static uint16_t calculate_stuffed_byte_count(const uint8_t* data, uint32_t len)
{
    uint32_t i = 0;
    uint32_t count = 0;

    for (i = 0; i < len; i++) {
        if ((data[i] == FRAME_BOUNDARY_OCTET) || (data[i] == CONTROL_ESCAPE_OCTET)) {
            count++;
        }
    }

    return count;
}

static hdlc_parse_result_t handle_state_idle(hdlc_t* p_hndl, uint8_t data)
{
    if (data == FRAME_BOUNDARY_OCTET) {
        p_hndl->state = HDLC_STATE_RECEIVING;
        p_hndl->buf_position = 0;
        return HDLC_PARSE_RESULT_PENDING;
    }

    return HDLC_PARSE_RESULT_IDLE;
}

static hdlc_parse_result_t handle_state_receiving(hdlc_t* p_hndl, uint8_t data)
{
    uint16_t fcs_received = 0;
    uint16_t fcs_calculated = 0;

    if (data == FRAME_BOUNDARY_OCTET) {
        p_hndl->state = HDLC_STATE_IDLE;

        fcs_received |= p_hndl->buf[p_hndl->buf_position - FCS_LEN];
        fcs_received |= ((uint16_t)p_hndl->buf[p_hndl->buf_position - FCS_LEN + 1]) << 8;

        fcs_calculated = calculate_fcs(FCS_INIT_VALUE, p_hndl->buf, p_hndl->buf_position - FCS_LEN);

        if (fcs_received == fcs_calculated) {
            return HDLC_PARSE_RESULT_COMPLETE;
        }
        else {
            return HDLC_PARSE_RESULT_IDLE;
        }
    }

    if (data == CONTROL_ESCAPE_OCTET) {
        p_hndl->state = HDLC_STATE_RECEIVING_STUFFED;
        return HDLC_PARSE_RESULT_PENDING;
    }

    p_hndl->buf[p_hndl->buf_position] = data;
    p_hndl->buf_position++;

    return HDLC_PARSE_RESULT_PENDING;
}

static hdlc_parse_result_t handle_state_receiving_stuffed(hdlc_t* p_hndl, uint8_t data)
{
    /* Force end. */
    if (data == FRAME_BOUNDARY_OCTET) {
        p_hndl->state = HDLC_STATE_IDLE;
        return HDLC_PARSE_RESULT_IDLE;
    }

    p_hndl->buf[p_hndl->buf_position++] = data ^ CONTROL_ESCAPE_INVERT;
    p_hndl->state = HDLC_STATE_RECEIVING;

    return HDLC_PARSE_RESULT_PENDING;
}



#ifdef HDLC_UNITTEST

#include <stdio.h>

#define ASSERT(x)                   do { \
                                        if (!x) { \
                                            printf ("Assert failed at %s:%d\r\n", __FILE__, __LINE__); \
                                            while(1); \
                                        } \
                                    } while (0);

static uint8_t hdlc_test_data[] = { 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F };
static uint32_t hdlc_test_data_len = sizeof(hdlc_test_data);

#define HDLC_TEST_BUF_LEN                       512U
static uint8_t hdlc_test_buf[HDLC_TEST_BUF_LEN];

static uint8_t hdlc_test_output_buffer[HDLC_TEST_BUF_LEN];

static uint8_t hdlc_test_input_buffer[HDLC_TEST_BUF_LEN];

void print_data(const uint8_t* data, uint32_t len)
{
    uint32_t i = 0;

    for (i = 0; i < len; i++) {
        printf("%.2x ", data[i]);
    }
    printf("\r\n");
}

void hdlc_test(void)
{
    uint32_t i = 0;
    uint32_t output_len = 0;
    hdlc_parse_result_t result = HDLC_PARSE_RESULT_IDLE;
    hdlc_t hdlc;

    printf("Start testing...\r\n");

    hdlc_init(&hdlc, hdlc_test_buf, HDLC_TEST_BUF_LEN);

    printf("Input data : ");
    print_data(hdlc_test_data, hdlc_test_data_len);

    printf("Write frame to buffer.\r\n");
    output_len = hdlc_write_output_frame(&hdlc, hdlc_test_data, hdlc_test_data_len, hdlc_test_output_buffer, HDLC_TEST_BUF_LEN);
    ASSERT(output_len > 0);

    printf("Output data : ");
    print_data(hdlc_test_output_buffer, output_len);

    for (i = 0; i < output_len; i++) {
        result = hdlc_input_byte(&hdlc, hdlc_test_output_buffer[i]);
    }
    ASSERT(result == HDLC_PARSE_RESULT_COMPLETE);
    ASSERT(hdlc_get_input_frame_len(&hdlc) == hdlc_test_data_len);

    hdlc_get_input_frame(&hdlc, hdlc_test_input_buffer, HDLC_TEST_BUF_LEN);

    for (i = 0; i < hdlc_get_input_frame_len(&hdlc); i++) {
        ASSERT(hdlc_test_input_buffer[i] == hdlc_test_data[i]);
    }

    printf("Done.\r\n");
}

#endif /* HDLC_UNITTEST */
