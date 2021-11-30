/*
 * Paket_Islemleri.c
 *
 *  Created on: 25 Mar 2020
 *      Author: Huseyin Koc
 *
 *	Paket ayirma ve birlestirme islemlerini icerir.
 */

#include "common.h"

void FLOAT32_ayir( uint8_t *veriler, uint8_t *paket_sayaci, const float veri_f32)
{
	veriler[(*paket_sayaci) + 0] = ((uint8_t*)&veri_f32)[0];
	veriler[(*paket_sayaci) + 1] = ((uint8_t*)&veri_f32)[1];
	veriler[(*paket_sayaci) + 2] = ((uint8_t*)&veri_f32)[2];
	veriler[(*paket_sayaci) + 3] = ((uint8_t*)&veri_f32)[3];

	*paket_sayaci += sizeof(float);
}

void FLOAT32_birlestir( const uint8_t *veriler, uint8_t *paket_sayaci, float *veri_f32)
{
	((uint8_t*)veri_f32)[0] = veriler[(*paket_sayaci) + 0];
	((uint8_t*)veri_f32)[1] = veriler[(*paket_sayaci) + 1];
	((uint8_t*)veri_f32)[2] = veriler[(*paket_sayaci) + 2];
	((uint8_t*)veri_f32)[3] = veriler[(*paket_sayaci) + 3];

	*paket_sayaci += sizeof(float);
}

void UINT32_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const uint32_t veri_u32)
{
	veriler[(*paket_sayaci) + 0] = ((uint8_t*)&veri_u32)[0];
	veriler[(*paket_sayaci) + 1] = ((uint8_t*)&veri_u32)[1];
	veriler[(*paket_sayaci) + 2] = ((uint8_t*)&veri_u32)[2];
	veriler[(*paket_sayaci) + 3] = ((uint8_t*)&veri_u32)[3];

	*paket_sayaci += sizeof(uint32_t);
}

void UINT32_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, uint32_t *veri_u32)
{
	((uint8_t*)veri_u32)[0] = veriler[(*paket_sayaci) + 0];
	((uint8_t*)veri_u32)[1] = veriler[(*paket_sayaci) + 1];
	((uint8_t*)veri_u32)[2] = veriler[(*paket_sayaci) + 2];
	((uint8_t*)veri_u32)[3] = veriler[(*paket_sayaci) + 3];

	*paket_sayaci += sizeof(uint32_t);
}

void INT32_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const int32_t veri_s32)
{
	veriler[(*paket_sayaci) + 0] = ((uint8_t*)&veri_s32)[0];
	veriler[(*paket_sayaci) + 1] = ((uint8_t*)&veri_s32)[1];
	veriler[(*paket_sayaci) + 2] = ((uint8_t*)&veri_s32)[2];
	veriler[(*paket_sayaci) + 3] = ((uint8_t*)&veri_s32)[3];

	*paket_sayaci += sizeof(int32_t);
}

void INT32_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, int32_t *veri_s32)
{
	((uint8_t*)veri_s32)[0] = veriler[(*paket_sayaci) + 0];
	((uint8_t*)veri_s32)[1] = veriler[(*paket_sayaci) + 1];
	((uint8_t*)veri_s32)[2] = veriler[(*paket_sayaci) + 2];
	((uint8_t*)veri_s32)[3] = veriler[(*paket_sayaci) + 3];

	*paket_sayaci += sizeof(int32_t);
}

void UINT16_ayir( uint8_t *veriler, uint8_t *paket_sayaci, const uint16_t veri_u16)
{
	veriler[(*paket_sayaci) + 0] = ((uint8_t*)&veri_u16)[0];
	veriler[(*paket_sayaci) + 1] = ((uint8_t*)&veri_u16)[1];

	*paket_sayaci += sizeof(uint16_t);
}

void UINT16_birlestir( const uint8_t *veriler, uint8_t *paket_sayaci,  uint16_t *veri_u16)
{
	((uint8_t*)veri_u16)[0] = veriler[(*paket_sayaci) + 0];
	((uint8_t*)veri_u16)[1] = veriler[(*paket_sayaci) + 1];

	*paket_sayaci += sizeof(uint16_t);
}

void INT16_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const int16_t veri_s16)
{
	veriler[(*paket_sayaci) + 0] = ((uint8_t*)&veri_s16)[0];
	veriler[(*paket_sayaci) + 1] = ((uint8_t*)&veri_s16)[1];

	*paket_sayaci += sizeof(int16_t);
}

void INT16_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, int16_t *veri_s16)
{
	((uint8_t*)veri_s16)[0] = veriler[(*paket_sayaci) + 0];
	((uint8_t*)veri_s16)[1] = veriler[(*paket_sayaci) + 1];

	*paket_sayaci += sizeof(int16_t);
}

void UINT8_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const uint8_t veri_u8)
{
	veriler[(*paket_sayaci) + 0] = ((uint8_t*)&veri_u8)[0];

	*paket_sayaci += sizeof(uint8_t);
}

void UINT8_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, uint8_t *veri_u8)
{
	((uint8_t*)veri_u8)[0] = veriler[(*paket_sayaci) + 0];

	*paket_sayaci += sizeof(uint8_t);
}

void INT8_ayir(uint8_t *veriler, uint8_t *paket_sayaci, const int8_t veri_s8)
{
	veriler[(*paket_sayaci) + 0] = ((uint8_t*)&veri_s8)[0];
	
	*paket_sayaci += sizeof(int8_t);
}

void INT8_birlestir(const uint8_t *veriler, uint8_t *paket_sayaci, int8_t *veri_s8)
{
	((uint8_t*)veri_s8)[0] = veriler[(*paket_sayaci) + 0];

	*paket_sayaci += sizeof(int8_t);
}

