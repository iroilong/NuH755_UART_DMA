# NuH755_UART_DMA

STM32H7 UART RX DMA experiment demonstrating a **high-throughput, producer-driven
ring buffer design** to prevent UART RX overrun.

This project explores how to reliably receive continuous UART data using
**circular DMA + software ring buffer**, even when the consumer side
(e.g. printf / VCP output) cannot keep up in real time.

---

## Motivation

On STM32H7, UART RX with DMA can easily overrun when:

- RX data rate is high
- RX data is forwarded via `printf()` / VCP
- Consumer-side processing is slower or blocking

This repository demonstrates a **producer-first design**:

> DMA always receives → software ring buffer absorbs bursts → consumer drains in chunks

---

## Key Concepts

- **Circular DMA RX buffer**
- **Producer-only ring buffer**
- **DMA write position tracking** via `__HAL_DMA_GET_COUNTER()`
- **Wrap-around safe handling**
- **Chunked consumer output** to avoid RX congestion

---

## Current Implementation

### RX Path (Producer)

- UART1 configured with **circular DMA**
- DMA RX buffer (`rx_dma_buf`) continuously filled by hardware
- DMA write position is computed as:
  ```c
  pos = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
