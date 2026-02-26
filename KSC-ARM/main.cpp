#include <stdint.h>

namespace {

constexpr uint32_t RCC_BASE = 0x40023800UL;
constexpr uint32_t GPIOA_BASE = 0x40020000UL;

struct RccRegs {
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t AHB3RSTR;
  uint32_t RESERVED0;
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t RESERVED1[2];
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  uint32_t RESERVED2;
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
};

struct GpioRegs {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
};

auto* const RCC = reinterpret_cast<RccRegs*>(RCC_BASE);
auto* const GPIOA = reinterpret_cast<GpioRegs*>(GPIOA_BASE);

constexpr uint32_t LED_PIN = 5U;
constexpr uint32_t LED_BIT = (1UL << LED_PIN);

void busy_wait(volatile uint32_t cycles) {
  while (cycles-- != 0U) {
    __asm volatile("nop");
  }
}

void init_led_pin_pa5() {
  RCC->AHB1ENR |= 0x01UL;
  (void)RCC->AHB1ENR;

  GPIOA->MODER &= ~(0x3UL << (LED_PIN * 2U));
  GPIOA->MODER |= (0x1UL << (LED_PIN * 2U));
  GPIOA->OTYPER &= ~LED_BIT;
  GPIOA->OSPEEDR |= (0x2UL << (LED_PIN * 2U));
  GPIOA->PUPDR &= ~(0x3UL << (LED_PIN * 2U));
}

}  // namespace

int main() {
  init_led_pin_pa5();

  while (true) {
    GPIOA->BSRR = LED_BIT;
    busy_wait(1800000U);

    GPIOA->BSRR = (LED_BIT << 16U);
    busy_wait(1800000U);
  }
}
