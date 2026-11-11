#ifndef RADIOLIB_HAL_STM32_H
#define RADIOLIB_HAL_STM32_H
/* RadioLib's hardware abstraction, implemented on the STM32 HAL and this board's pin labels. */
#include "RadioLib.h"
#include "main.h"
#include "spi.h"
#include "app_time.h"

/* RadioLib names pins with plain integers; these are our own ids, mapped to real pins in lookup(). */
enum : uint32_t { RL_PIN_CS = 0, RL_PIN_IRQ = 1, RL_PIN_RST = 2, RL_PIN_BUSY = 3 };

class StmHal : public RadioLibHal {
public:
    explicit StmHal(SPI_HandleTypeDef *spi)
        : RadioLibHal(0 /*input*/, 1 /*output*/, 0 /*low*/, 1 /*high*/, 1 /*rising*/, 2 /*falling*/),
          spi_(spi) {}

    /* Pin directions and the EXTI line are configured by CubeMX, so these are intentionally empty. */
    void pinMode(uint32_t, uint32_t) override {}
    void detachInterrupt(uint32_t) override { cb_ = nullptr; }
    void attachInterrupt(uint32_t, void (*cb)(void), uint32_t) override { cb_ = cb; }
    long pulseIn(uint32_t, uint32_t, RadioLibTime_t) override { return 0; }
    void spiBegin() override {}
    void spiBeginTransaction() override {}
    void spiEndTransaction() override {}
    void spiEnd() override {}

    void digitalWrite(uint32_t pin, uint32_t v) override {
        GPIO_TypeDef *port; uint16_t p;
        if (!lookup(pin, &port, &p)) return;
        HAL_GPIO_WritePin(port, p, v ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    uint32_t digitalRead(uint32_t pin) override {
        GPIO_TypeDef *port; uint16_t p;
        if (!lookup(pin, &port, &p)) return 0;
        return HAL_GPIO_ReadPin(port, p) == GPIO_PIN_SET;
    }
    void delay(RadioLibTime_t ms) override { HAL_Delay((uint32_t)ms); }
    void delayMicroseconds(RadioLibTime_t us) override {
        uint32_t start = DWT->CYCCNT, ticks = (uint32_t)us * (SystemCoreClock / 1000000u);
        while ((uint32_t)(DWT->CYCCNT - start) < ticks) {}
    }
    RadioLibTime_t millis() override { return HAL_GetTick(); }
    RadioLibTime_t micros() override { return (RadioLibTime_t)time_us(); }

    void spiTransfer(uint8_t *out, size_t len, uint8_t *in) override {
        if (out && in)  HAL_SPI_TransmitReceive(spi_, out, in, (uint16_t)len, 10);
        else if (out)   HAL_SPI_Transmit(spi_, out, (uint16_t)len, 10);
        else            HAL_SPI_Receive(spi_, in, (uint16_t)len, 10);
    }

    /* Called from the EXTI interrupt when IRQ_LORA fires: runs RadioLib's registered action. */
    void irqFired() { if (cb_) cb_(); }

private:
    static bool lookup(uint32_t pin, GPIO_TypeDef **port, uint16_t *p) {
        switch (pin) {
        case RL_PIN_CS:   *port = CS_LORA_GPIO_Port;    *p = CS_LORA_Pin;    return true;
        case RL_PIN_RST:  *port = RESET_LORA_GPIO_Port; *p = RESET_LORA_Pin; return true;
        case RL_PIN_BUSY: *port = BUSY_LORA_GPIO_Port;  *p = BUSY_LORA_Pin;  return true;
        case RL_PIN_IRQ:  *port = IRQ_LORA_GPIO_Port;   *p = IRQ_LORA_Pin;   return true;
        default: return false;
        }
    }
    SPI_HandleTypeDef *spi_;
    void (*cb_)(void) = nullptr;
};
#endif
