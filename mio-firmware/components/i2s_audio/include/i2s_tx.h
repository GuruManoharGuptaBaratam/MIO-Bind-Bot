#ifndef I2S_TX_H
#define I2S_TX_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_i2s_master_tx(void);
void stream_audio_over_i2s(const uint8_t *pcm_data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // I2S_TX_H