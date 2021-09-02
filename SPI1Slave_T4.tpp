#include <SPI1Slave_T4.h>
#include "Arduino.h"
#include "SPI.h"

// 48.4.1.1 LPSPI Memory map. Base address LPSPI1: 4039_4000h. Page 2810
#define SLAVE_PORT_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x40394000 + (0x4000 * _portnum)))

// 11.7.311 LPSPI1_PCS0_SELECT_INPUT DAISY Register (IOMUXC_LPSPI1_PCS0_SELECT_INPUT) 401F_84ECh (Baseaddr * (_portnum * 16K))
#define SLAVE_PINS_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x401F84EC + (_portnum * 0x10)))

#define SLAVE_CR spiAddr[4]     // 48.4.1.4   Control register. page 2813
#define SLAVE_SR spiAddr[5]     // 48.4.1.5   Status Register, page 2814
#define SLAVE_IER spiAddr[6]    // 48.4.1.6   Interrupt Enable Register, 2816
#define SLAVE_CFGR0 spiAddr[8]  // 48.4.1.8   Configuration Register 0, page 2818
#define SLAVE_CFGR1 spiAddr[9]  // 48.4.1.9   Configuration Register 1, page 2819
#define SLAVE_FCR spiAddr[22]   // 48.4.1.13  FIFO Control Register. page 2825
                                // 48.4.1.15  Transmit Command Register, page 2827
#define SLAVE_TCR_REFRESH spiAddr[24] = (0UL << 27) | LPSPI_TCR_FRAMESZ(bits - 1) // May be choosing the wrong CS here. Can it share with LPSPI4?
#define SLAVE_TDR spiAddr[25]   // 48.4.1.16  Transmit Data Register (TDR), page 2830
#define SLAVE_RDR spiAddr[29]   // 48.4.1.18  Receive Data Register, page 2831

 
void lpspi3_slave_isr() {
  _LPSPI3->SLAVE_ISR();
}


SPI1Slave_T4_FUNC SPI1Slave_T4_OPT::SPI1Slave_T4() {
  if ( port == &SPI1 ) {
    _LPSPI3 = this;
    _portnum = 2;               // According to: https://forum.pjrc.com/threads/61234-Teensy-4-1-and-SPI1 
                                // _LPSPI4= SPI= portnum 3, _LPSPI3= SPI1= portnum 2, _LPSPI2= SPI2= portnum 1, _LPSPI1= SPI4= portnum 0
                                // Confused yet?

    CCM_CCGR1 |= (3UL << 6);    // Clock gating register. CG3=6, CG2=4, CG1=2, CG0=0. Oddly, changing from CG3 to CG2 causes hang on init.
                                // There appears to be a need for corrdination between setting this register and CCM_CBCMR, as per the thread:
                                // https://forum.pjrc.com/threads/59254-SPI-Slave-Mode-on-Teensy-4
                                // "CCM_CBCMR |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON); //Clock reaktivieren (reactivate)". Note he also appears to
                                // set SION on the CS pin as we also did below.
    nvic_irq = 32 + _portnum;
    _VectorsRam[16 + nvic_irq] = lpspi3_slave_isr;

    /* Alternate pins not broken out on Teensy 4.0/4.1 for LPSPI4 */
    SLAVE_PINS_ADDR;
    spiAddr[0] = 0; // IOMUXC_LPSPI3_PCS0_SELECT_INPUT (401F_850Ch) GPIO_AD_B0_03_ALT7 
    spiAddr[1] = 1; // IOMUXC_LPSPI3_SCK_SELECT_INPUT (401F_8510h) GPIO_AD_B1_15_ALT2
    spiAddr[2] = 0; // IOMUXC_LPSPI3_SDI_SELECT_INPUT (401F_8514h) GPIO_AD_B0_02_ALT7
    spiAddr[3] = 1; // IOMUXC_LPSPI3_SDO_SELECT_INPUT (401F_8518h) GPIO_AD_B1_14_ALT2
    // These are the primary SPI mux control registers.
    // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // LPSPI4 SCK (CLK) 13 ALT3 
    // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; // LPSPI4 SDI (MISO) 12 
    // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; // LPSPI4 SDO (MOSI) 11 ALT3
    // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // LPSPI4 PCS0 (CS) 10 ALT3
    // SPI1
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 0x17; // LPSPI3_PCS0 (CS1) 0 ALT7 + SION. See: https://forum.pjrc.com/archive/index.php/t-59893.html
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 0x12; // LPSPI3_SCK1 (CLK) 27 ALT2
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 0x17; // LPSPI3_SDI (MISO1) 1 ALT7
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 0x12; // LPSPI3_SDO (MOSI1) 26 ALT2

  } 
}


// "disables the slave output pin so it listens to traffic only." See: https://forum.pjrc.com/threads/66389-SPISlave_T4
SPI1Slave_T4_FUNC void SPI1Slave_T4_OPT::swapPins(bool enable) {
  SLAVE_PORT_ADDR;
  SLAVE_CR &= ~LPSPI_CR_MEN; // Disable Module 
  SLAVE_CFGR1 = (SLAVE_CFGR1 & 0xFCFFFFFF) | (enable) ? (3UL << 24) : (0UL << 24);  // PINCFG
  SLAVE_CR |= LPSPI_CR_MEN; // Enable Module 
  if ( sniffer_enabled ) sniffer();
}


SPI1Slave_T4_FUNC void SPI1Slave_T4_OPT::sniffer(bool enable) {
  SLAVE_PORT_ADDR;
  sniffer_enabled = enable;
  if ( port == &SPI1 ) {
    if ( sniffer_enabled ) {
      if ( SLAVE_CFGR1 & (3UL << 24) ) { /* if pins are swapped */
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // LPSPI4 SCK (CLK) 13 ALT3
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x0; // LPSPI4 SDI (MISO) 12 ALT0 LCD_ENABLE? See comment above swapPins()
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; // LPSPI4 SDO (MOSI) 11 ALT3
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // LPSPI4 PCS0 (CS) 10 ALT3
          IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 0x17; // LPSPI3_PCS0 (CS1) 0 ALT7 + SION
          IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 0x12; // LPSPI3_SCK1 (CLK) 27 ALT2
          IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 0x17; // LPSPI3_SDI (MISO1) 1 ALT7
          IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 0x12; // LPSPI3_SDO (MOSI1) 26 ALT2
      }
      else {
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // LPSPI4 SCK (CLK) 13 ALT3
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; // LPSPI4 SDI (MISO) 12 ALT3
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x0; // LPSPI4 SDO (MOSI) 11 ALT0 LCD_HSYNC? See comment above swapPins()
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // LPSPI4 PCS0 (CS) 10 ALT3
          IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 0x17; // LPSPI3_PCS0 (CS1) 0 ALT7 + SION
          IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 0x12; // LPSPI3_SCK1 (CLK) 27 ALT2  
          IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 0x17; // LPSPI3_SDI (MISO1) 1 ALT7 
          IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 0x12; // LPSPI3_SDO (MOSI1) 26 ALT2 
      }
    }
    else {
      // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // LPSPI4 SCK (CLK) ALT3
      // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; // LPSPI4 SDI (MISO) ALT3
      // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; // LPSPI4 SDO (MOSI) ALT3
      // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // LPSPI4 PCS0 (CS) ALT3
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 0x17; // LPSPI3_PCS0 (CS1) 0 ALT7 + SION
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 0x12; // LPSPI3_SCK1 (CLK) 27 ALT2 
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 0x17; // LPSPI3_SDI (MISO1) 1 ALT7 
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 0x12; // LPSPI3_SDO (MOSI1) 26 ALT2 
    }
  }
}


SPI1Slave_T4_FUNC bool SPI1Slave_T4_OPT::active() {
  SLAVE_PORT_ADDR;
  return ( !(SLAVE_SR & (1UL << 9)) ) ? 1 : 0;
}


SPI1Slave_T4_FUNC bool SPI1Slave_T4_OPT::available() {
  SLAVE_PORT_ADDR;
  return ( (SLAVE_SR & (1UL << 8)) ) ? 1 : 0;
}


SPI1Slave_T4_FUNC void SPI1Slave_T4_OPT::pushr(uint32_t data) {
  SLAVE_PORT_ADDR;
  SLAVE_TDR = data;
}


SPI1Slave_T4_FUNC uint32_t SPI1Slave_T4_OPT::popr() {
  SLAVE_PORT_ADDR;
  uint32_t data = SLAVE_RDR;
  SLAVE_SR = (1UL << 8); /* Clear WCF */
  return data;
}


SPI1Slave_T4_FUNC void SPI1Slave_T4_OPT::SLAVE_ISR() {

  SLAVE_PORT_ADDR;

  if ( _spihandler ) {
    _spihandler();
    SLAVE_SR = 0x3F00;
    asm volatile ("dsb");
    return;
  }

  while ( !(SLAVE_SR & (1UL << 9)) ) { /* FCF: Frame Complete Flag, set when PCS deasserts */
    if ( SLAVE_SR & (1UL << 11) ) { /* transmit error, clear flag, check cabling */
      SLAVE_SR = (1UL << 11);
      transmit_errors++;
    }
    if ( (SLAVE_SR & (1UL << 8)) ) { /* WCF set */
      uint32_t val = SLAVE_RDR;
      Serial.print(val); Serial.print(" ");
      SLAVE_TDR = val;
      SLAVE_SR = (1UL << 8); /* Clear WCF */
    }
  }
  Serial.println();
  SLAVE_SR = 0x3F00; /* Clear remaining flags on exit */
  asm volatile ("dsb");
}


SPI1Slave_T4_FUNC void SPI1Slave_T4_OPT::begin() {
  SLAVE_PORT_ADDR;
  SLAVE_CR = LPSPI_CR_RST;// Reset Module
  SLAVE_CR = 0;           // Disable Module
  SLAVE_FCR = 0;          // x10001; // 1x watermark for RX and TX
  SLAVE_IER = 0x1;        // RX Interrupt - datasheet says 0x1 is TX interrupt. RX is 0x2.
  SLAVE_CFGR0 = 0;        // Verify HRSEL. Should be 1?
  SLAVE_CFGR1 = 0;        // slave, sample on SCK rising edge, !autoPCS (must raise CS between frames), FIFO will stall, CS active low, match disabled, 
  SLAVE_CR |= LPSPI_CR_MEN | LPSPI_CR_DBGEN; /* Enable Module, Debug Mode */
  SLAVE_SR = 0x3F00;      // Clear status register
  SLAVE_TCR_REFRESH;
  SLAVE_TDR = 0x0;        // dummy data, must populate initial TX slot
  NVIC_ENABLE_IRQ(nvic_irq);
  NVIC_SET_PRIORITY(nvic_irq, 1);
}