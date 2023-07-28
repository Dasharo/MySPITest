#include "main.h"
#include "spi.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define STATE_WAIT_HEADER 0
#define STATE_WAIT_STATE 1
#define STATE_WAIT_STATE_LAST 2
#define STATE_PAYLOAD_TRANSFER 3

static __attribute__((aligned(32))) uint8_t ff_buffer[64];
static __attribute__((aligned(32))) uint8_t waitstate_insert[4] = {0xff, 0xff, 0xff, 0xfe};
static __attribute__((aligned(32))) uint8_t waitstate_hold[1] = {0x00};

static __attribute__((aligned(32))) uint8_t waitstate_cancel[1] = {0x01};
static __attribute__((aligned(32))) uint8_t trash[4] = {0};
static __attribute__((aligned(32))) uint8_t header[4] = {0};
static uint8_t state = STATE_WAIT_HEADER;

static __attribute__((aligned(32))) uint8_t scratch_buffer[64] = {0};

static uint8_t transfer_is_read = 0;
static uint32_t transfer_length = 0;

static void txdma_complete(DMA_HandleTypeDef *hdma)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);

	return;

	switch (state) {
	case STATE_WAIT_HEADER:
		// Wait state got inserted while reading header.
		HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)waitstate_cancel, (uint32_t)&hspi->Instance->DR, sizeof waitstate_cancel);

		// We don't care what host sends during wait state, but we start DMA anyway to avoid overrun errors.
		HAL_DMA_PollForTransfer(hspi->hdmarx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
		HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)trash, sizeof trash);

		transfer_is_read = !!(header[0] & (1 << 7));
		transfer_length = (header[0] & 0x3f) + 1;

		state = STATE_WAIT_STATE_LAST;
		break;

	case STATE_WAIT_STATE_LAST:
		if (transfer_is_read) {
			HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)scratch_buffer, (uint32_t)&hspi->Instance->DR, transfer_length);
			HAL_DMA_PollForTransfer(hspi->hdmarx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
			HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)ff_buffer, transfer_length);
		} else {
			HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)ff_buffer, (uint32_t)&hspi->Instance->DR, transfer_length);
			HAL_DMA_PollForTransfer(hspi->hdmarx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
			HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)scratch_buffer, transfer_length);
		}

		state = STATE_PAYLOAD_TRANSFER;
		break;

	case STATE_PAYLOAD_TRANSFER:
		HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)waitstate_insert, (uint32_t)&hspi->Instance->DR, sizeof waitstate_insert);
		HAL_DMA_PollForTransfer(hspi->hdmarx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
		HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)header, sizeof header);

		state = STATE_WAIT_HEADER;
		break;
	}
}

void app_main() {
	SPI_HandleTypeDef *hspi = &hspi2;

	assert_param(IS_SPI_DMA_HANDLE(hspi->hdmarx));
	assert_param(IS_SPI_DMA_HANDLE(hspi->hdmatx));
	assert(hspi->hdmatx->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE);
	assert(hspi->hdmarx->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE);

	memset(ff_buffer, 0xff, sizeof ff_buffer);

	//hspi->hdmarx->XferCpltCallback = rxdma_complete;
	hspi->hdmatx->XferCpltCallback = txdma_complete;

	SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
	CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_LDMARX | SPI_CR2_LDMATX);

	hspi->hdmarx->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hspi->hdmarx->ChannelIndex & 0x1CU))
			| (DMA_ISR_GIF1 << (hspi->hdmatx->ChannelIndex & 0x1CU));

	hspi->hdmarx->Instance->CNDTR = sizeof header;
	hspi->hdmarx->Instance->CPAR = (uint32_t)&hspi->Instance->DR;
	hspi->hdmarx->Instance->CMAR = (uint32_t)header;
	__HAL_DMA_ENABLE_IT(hspi->hdmarx, DMA_IT_TC | DMA_IT_TE);
	__HAL_DMA_ENABLE(hspi->hdmarx);
	SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

	//HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)waitstate_insert, (uint32_t)&hspi->Instance->DR, sizeof waitstate_insert);
	hspi->hdmatx->Instance->CNDTR = sizeof waitstate_insert;
	hspi->hdmatx->Instance->CPAR = (uint32_t)&hspi->Instance->DR;
	hspi->hdmatx->Instance->CMAR = (uint32_t)header;
	__HAL_DMA_ENABLE_IT(hspi->hdmatx, DMA_IT_TC | DMA_IT_TE);
	__HAL_DMA_ENABLE(hspi->hdmatx);
	SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);

	__HAL_SPI_ENABLE(hspi);
}
