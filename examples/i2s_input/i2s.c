#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "esp/slc.h"
#include "esp/i2s_regs.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>

#include "lwip/sockets.h"
#include "ssid_config.h"

#ifndef i2c_bbpll
#define i2c_bbpll 0x67
#define i2c_bbpll_en_audio_clock_out 4
#define i2c_bbpll_en_audio_clock_out_msb 7
#define i2c_bbpll_en_audio_clock_out_lsb 7
#define i2c_bbpll_hostid 4

#define i2c_writeReg_Mask(block, host_id, reg_add, Msb, Lsb, indata) \
    sdk_rom_i2c_writeReg_Mask(block, host_id, reg_add, Msb, Lsb, indata)
#define i2c_readReg_Mask(block, host_id, reg_add, Msb, Lsb) \
    sdk_rom_i2c_readReg_Mask(block, host_id, reg_add, Msb, Lsb)
#define i2c_writeReg_Mask_def(block, reg_add, indata) \
      i2c_writeReg_Mask(block, block##_hostid, reg_add, reg_add##_msb, \
                        reg_add##_lsb, indata)
#define i2c_readReg_Mask_def(block, reg_add) \
      i2c_readReg_Mask(block, block##_hostid, reg_add, reg_add##_msb, \
                       reg_add##_lsb)
#endif

#define I2S_RESET_MASK (I2S_CONF_RX_FIFO_RESET | I2S_CONF_TX_FIFO_RESET \
                       | I2S_CONF_RX_RESET | I2S_CONF_TX_RESET)

// SLC DMA

#define I2SNDMABUF 16
#define I2SBUFLEN (32*2)

static struct SLCDescriptor i2s_desc[I2SNDMABUF];
static uint32_t i2s_buf[I2SNDMABUF*I2SBUFLEN];
static xQueueHandle dmaqueue;

static void slc_isr(void)
{
    portBASE_TYPE awoken = 0;
    uint32_t status = SLC.INT_STATUS;

    // printf("st %08x\n", status);
    if (status == 0) {
        return;
    }
    SLC.INT_CLEAR = 0xffffffff;
    if (status & SLC_INT_STATUS_TX_EOF) {
        struct SLCDescriptor *desc = (void*)SLC.TX_EOF_DESCRIPTOR_ADDR;
        xQueueSendFromISR(dmaqueue, (void*)&desc->buf_ptr, &awoken);
#if 1
        desc->flags &= ~SLC_DESCRIPTOR_FLAGS_EOF;
        desc->flags |= SLC_DESCRIPTOR_FLAGS_OWNER;
#endif
    }
    portEND_SWITCHING_ISR(awoken);
}

static int next(int n)
{
    if (n+1 < I2SNDMABUF) {
        return n+1;
    }
    return 0;
} 

void i2s_init(void)
{
    // reset DMA
    SLC.CONF0 |= (SLC_CONF0_TX_LOOP_TEST
                  | SLC_CONF0_RX_LINK_RESET | SLC_CONF0_TX_LINK_RESET);
    SLC.CONF0 &= ~(SLC_CONF0_RX_LINK_RESET | SLC_CONF0_TX_LINK_RESET);
    // clear DMA int flags
    SLC.INT_CLEAR = 0xffffffff;
    SLC.INT_CLEAR = 0;
    // enable and configure DMA. TX&RX are named from the DMA point of view
    SLC.CONF0 &= ~(SLC_CONF0_MODE_M << SLC_CONF0_MODE_S);
    SLC.CONF0 |= (0x01 << SLC_CONF0_MODE_S);
    SLC.RX_DESCRIPTOR_CONF |= (SLC_RX_DESCRIPTOR_CONF_INFOR_NO_REPLACE
                               | SLC_RX_DESCRIPTOR_CONF_TOKEN_NO_REPLACE);
    SLC.RX_DESCRIPTOR_CONF &= ~(SLC_RX_DESCRIPTOR_CONF_RX_FILL_ENABLE
                                | SLC_RX_DESCRIPTOR_CONF_RX_EOF_MODE
                                | SLC_RX_DESCRIPTOR_CONF_RX_FILL_MODE);

    for (int i = 0; i < I2SNDMABUF; i++) {
        uint32_t size = I2SBUFLEN * sizeof(uint32_t);
        i2s_desc[i].flags = SLC_DESCRIPTOR_FLAGS(size, size, 0, 1, 1);
        i2s_desc[i].buf_ptr = (uint32_t)&i2s_buf[I2SBUFLEN*i];
        i2s_desc[i].next_link_ptr = (uint32_t)&i2s_desc[next(i)];
    }

    SLC.TX_LINK &= ~SLC_TX_LINK_DESCRIPTOR_ADDR_M;
    SLC.TX_LINK |= ((uint32_t)&i2s_desc[0]) & SLC_TX_LINK_DESCRIPTOR_ADDR_M;
    SLC.RX_LINK &= ~SLC_RX_LINK_DESCRIPTOR_ADDR_M;
    SLC.RX_LINK |= ((uint32_t)&i2s_desc[0]) & SLC_RX_LINK_DESCRIPTOR_ADDR_M;

    _xt_isr_attach(INUM_SLC, slc_isr);
    SLC.INT_ENABLE = (SLC_INT_STATUS_TX_DSCR_EMPTY
                      | SLC_INT_ENABLE_TX_DSCR_ERROR
                      | SLC_INT_ENABLE_TX_DONE
                      | SLC_INT_ENABLE_TX_EOF
                      | SLC_INT_STATUS_TX_OVERFLOW);
    SLC.INT_CLEAR = 0xffffffff;
    _xt_isr_unmask(1<<INUM_SLC);

    dmaqueue=xQueueCreate(I2SNDMABUF, sizeof(int*));
    SLC.TX_LINK |= SLC_TX_LINK_START;

    // configure pins
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_I2SI_BCK);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);

    // enable clock to i2s module
    i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1);

    I2S.CONF &= ~I2S_RESET_MASK;
    I2S.CONF |= I2S_RESET_MASK;
    I2S.CONF &= ~I2S_RESET_MASK;

    I2S.FIFO_CONF &= ~(I2S_FIFO_CONF_DESCRIPTOR_ENABLE
                        | (I2S_FIFO_CONF_RX_FIFO_MOD_M
                           << I2S_FIFO_CONF_RX_FIFO_MOD_S)
                        | (I2S_FIFO_CONF_TX_FIFO_MOD_M
                           << I2S_FIFO_CONF_TX_FIFO_MOD_S));
    // 24-bit per channel
    I2S.FIFO_CONF |=  (I2S_FIFO_CONF_DESCRIPTOR_ENABLE
                       | (3 << I2S_FIFO_CONF_RX_FIFO_MOD_S));
    I2S.RX_EOF_NUM = I2SBUFLEN;
    I2S.CONF_CHANNELS &= ~((I2S_CONF_CHANNELS_RX_CHANNEL_MOD_M
                            << I2S_CONF_CHANNELS_RX_CHANNEL_MOD_S)
                           | (I2S_CONF_CHANNELS_TX_CHANNEL_MOD_M
                              << I2S_CONF_CHANNELS_TX_CHANNEL_MOD_S));
    // left channel only
    I2S.CONF_CHANNELS |= (2 << I2S_CONF_CHANNELS_RX_CHANNEL_MOD_S);
    I2S.INT_CLEAR |= (I2S_INT_CLEAR_TX_REMPTY
                       | I2S_INT_CLEAR_TX_WFULL
                       | I2S_INT_CLEAR_RX_WFULL
                       | I2S_INT_CLEAR_TX_PUT_DATA
                       | I2S_INT_CLEAR_RX_TAKE_DATA);
    I2S.INT_CLEAR &= ~(I2S_INT_CLEAR_TX_REMPTY
                        | I2S_INT_CLEAR_TX_WFULL
                        | I2S_INT_CLEAR_RX_WFULL
                        | I2S_INT_CLEAR_TX_PUT_DATA
                        | I2S_INT_CLEAR_RX_TAKE_DATA);
    I2S.CONF &= ~(I2S_CONF_RX_SLAVE_MOD
                  | I2S_CONF_TX_SLAVE_MOD
                  | (I2S_CONF_BITS_MOD_M << I2S_CONF_BITS_MOD_S)
                  | (I2S_CONF_BCK_DIV_M << I2S_CONF_BCK_DIV_S)
                  | (I2S_CONF_CLKM_DIV_M << I2S_CONF_CLKM_DIV_S));
    // WS=160MHz/(BCK_DIV*CLM_DIV*24*2)
    // use BCK_DIV=4 and CLKM_DIV=52 for WS=16025Hz
    I2S.CONF |= (I2S_CONF_MSB_RIGHT
                  | I2S_CONF_RX_MSB_SHIFT
                  | (8 << I2S_CONF_BITS_MOD_S)
                  | (4 << I2S_CONF_BCK_DIV_S)
                  | (52 << I2S_CONF_CLKM_DIV_S));
    
    I2S.INT_CLEAR |= (I2S_INT_CLEAR_TX_REMPTY
                       | I2S_INT_CLEAR_TX_WFULL
                       | I2S_INT_CLEAR_RX_WFULL
                       | I2S_INT_CLEAR_TX_PUT_DATA
                       | I2S_INT_CLEAR_RX_TAKE_DATA);
    I2S.INT_CLEAR &= ~(I2S_INT_CLEAR_TX_REMPTY
                        | I2S_INT_CLEAR_TX_WFULL
                        | I2S_INT_CLEAR_RX_WFULL
                        | I2S_INT_CLEAR_TX_PUT_DATA
                        | I2S_INT_CLEAR_RX_TAKE_DATA);

    // start i2s
    I2S.CONF |= I2S_CONF_RX_START;
}

static uint32_t *curbuf = NULL;

void i2s_task(void *pvParameters)
{
    int sockfd = (int)pvParameters;
#if 1
    gpio_enable (5, GPIO_INPUT);
#endif

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    // required to call wifi_set_opmode before station_set_config
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
    sdk_wifi_station_set_auto_connect(1);

    while (1) {
        xQueueReceive(dmaqueue, &curbuf, portMAX_DELAY);
#if 1
        if (gpio_read (5) == 1) {
            continue;
        }
#endif
        int n = send(sockfd, curbuf, I2SBUFLEN * sizeof(uint32_t), 0);
        if (n < 0) {
        }
    }
}
