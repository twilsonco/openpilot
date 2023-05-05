// TACH interrupt handler
void EXTI2_IRQ_Handler(void) {
  volatile unsigned int pr = EXTI->PR1 & (1U << 2);
  if ((pr & (1U << 2)) != 0U) {
    fan_state.tach_counter++;
  }
  EXTI->PR1 = (1U << 2);
}
void fan_init(void){ }
