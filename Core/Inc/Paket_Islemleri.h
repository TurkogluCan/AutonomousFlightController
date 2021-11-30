/*
 * Paket_Islemleri.h
 *
 *  Created on: 25 Mar 2020
 *      Author: Huseyin Koc
 */

#ifndef PAKET_ISLEMLERI_H_
#define PAKET_ISLEMLERI_H_

void FLOAT32_ayir( uint8_t *veriler, uint8_t *paket_sayaci, const float veri_f32);
void FLOAT32_birlestir( const uint8_t *veriler, uint8_t *paket_sayaci, float *veri_f32);
void UINT32_ayir(uint8_t *veriler, uint8_t *paket_sayaci, uint32_t const veri_u32);
void UINT32_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, uint32_t *veri_u32);
void INT32_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const int32_t veri_s32);
void INT32_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, int32_t *veri_s32);
void UINT16_ayir( uint8_t *veriler, uint8_t *paket_sayaci, const uint16_t veri_u16);
void UINT16_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci,  uint16_t *veri_u16);
void INT16_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const int16_t veri_s16);
void INT16_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, int16_t *veri_s16);
void UINT8_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const uint8_t veri_u8);
void UINT8_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, uint8_t *veri_u8);
void INT8_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const int8_t veri_s8);
void INT8_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, int8_t *veri_s8);
#endif /* PAKET_ISLEMLERI_H_ */
