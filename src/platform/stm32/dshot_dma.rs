use log::info;

// DMA1 for TIM3 (MOTOR1/2 on PC8/PC9)
const DMA1_BASE: u32 = 0x4002_6000;
const DMA1_LISR: u32 = DMA1_BASE + 0x00;
const DMA1_HISR: u32 = DMA1_BASE + 0x04;
const DMA1_LIFCR: u32 = DMA1_BASE + 0x08;
const DMA1_HIFCR: u32 = DMA1_BASE + 0x0C;

// DMA2 for TIM1 (MOTOR3/4 on PA8/PA9)
const DMA2_BASE: u32 = 0x4002_6400;
const DMA2_LISR: u32 = DMA2_BASE + 0x00;
const DMA2_LIFCR: u32 = DMA2_BASE + 0x08;

// DMA1 Stream flags
const DMA1_TCIF7: u32 = 1 << 27;  // Stream7 transfer complete (HISR)
const DMA1_TEIF7: u32 = 1 << 25;  // Stream7 transfer error
const DMA1_TCIF2: u32 = 1 << 21;  // Stream2 transfer complete (LISR)
const DMA1_TEIF2: u32 = 1 << 19;  // Stream2 transfer error
const DMA1_ALL_FLAGS7: u32 = 0x0F400000; // Clear all Stream7 flags
const DMA1_ALL_FLAGS2: u32 = 0x003F0000; // Clear all Stream2 flags

// DMA2 Stream flags (Stream1 and Stream2 are in LISR)
const DMA2_TCIF1: u32 = 1 << 11;  // Stream1 transfer complete
const DMA2_TEIF1: u32 = 1 << 9;   // Stream1 transfer error
const DMA2_TCIF2: u32 = 1 << 21;  // Stream2 transfer complete
const DMA2_TEIF2: u32 = 1 << 19;  // Stream2 transfer error
const DMA2_ALL_FLAGS1: u32 = 0x00000F40; // Clear all Stream1 flags
const DMA2_ALL_FLAGS2: u32 = 0x003F0000; // Clear all Stream2 flags

// DMA stream register offsets
const DMA_SxCR: u32 = 0x00;
const DMA_SxNDTR: u32 = 0x04;
const DMA_SxPAR: u32 = 0x08;
const DMA_SxM0AR: u32 = 0x0C;
const DMA_SxFCR: u32 = 0x14;

// DMA stream base addresses
const DMA1_STREAM7_BASE: u32 = DMA1_BASE + 0x0B8;  // TIM3_CH3
const DMA1_STREAM2_BASE: u32 = DMA1_BASE + 0x040;  // TIM3_CH4
const DMA2_STREAM1_BASE: u32 = DMA2_BASE + 0x028;  // TIM1_CH1
const DMA2_STREAM2_BASE: u32 = DMA2_BASE + 0x040;  // TIM1_CH2

// DMA control register bits
const DMA_CR_EN: u32 = 1 << 0;
const DMA_CR_DIR_M2P: u32 = 1 << 6;  // Memory to peripheral
const DMA_CR_MINC: u32 = 1 << 10;    // Memory increment
const DMA_CR_PSIZE_32: u32 = 2 << 11; // Peripheral size 32-bit
const DMA_CR_MSIZE_32: u32 = 2 << 13; // Memory size 32-bit

// DMA channel selection (bits 25-27)
const CHSEL_CH5: u32 = 5 << 25;  // Channel 5 for TIM3
const CHSEL_CH6: u32 = 6 << 25;  // Channel 6 for TIM1

pub const DSHOT_DMA_BUFFER_SIZE: usize = 18;

// TIM3 registers (APB1, for MOTOR1/2)
const TIM3_BASE: u32 = 0x4000_0400;
const TIM3_CR1: u32 = TIM3_BASE + 0x00;
const TIM3_DIER: u32 = TIM3_BASE + 0x0C;
const TIM3_EGR: u32 = TIM3_BASE + 0x14;
const TIM3_CCMR2: u32 = TIM3_BASE + 0x1C;
const TIM3_CCER: u32 = TIM3_BASE + 0x20;
const TIM3_PSC: u32 = TIM3_BASE + 0x28;
const TIM3_ARR: u32 = TIM3_BASE + 0x2C;
const TIM3_CCR3: u32 = TIM3_BASE + 0x3C;
const TIM3_CCR4: u32 = TIM3_BASE + 0x40;

// TIM1 registers (APB2, for MOTOR3/4) - advanced timer
const TIM1_BASE: u32 = 0x4001_0000;
const TIM1_CR1: u32 = TIM1_BASE + 0x00;
const TIM1_DIER: u32 = TIM1_BASE + 0x0C;
const TIM1_EGR: u32 = TIM1_BASE + 0x14;
const TIM1_CCMR1: u32 = TIM1_BASE + 0x18;
const TIM1_CCER: u32 = TIM1_BASE + 0x20;
const TIM1_PSC: u32 = TIM1_BASE + 0x28;
const TIM1_ARR: u32 = TIM1_BASE + 0x2C;
const TIM1_CCR1: u32 = TIM1_BASE + 0x34;
const TIM1_CCR2: u32 = TIM1_BASE + 0x38;
const TIM1_BDTR: u32 = TIM1_BASE + 0x44;  // Break and dead-time register

// RCC registers
const RCC_AHB1ENR: u32 = 0x4002_3830;
const RCC_APB1ENR: u32 = 0x4002_3840;
const RCC_APB2ENR: u32 = 0x4002_3844;

// GPIO registers
const GPIOA_BASE: u32 = 0x4002_0000;
const GPIOC_BASE: u32 = 0x4002_0800;

const CR1_CEN: u32 = 1 << 0;
const CR1_URS: u32 = 1 << 2;

// BDTR MOE bit (Main Output Enable) - required for TIM1 advanced timer
const BDTR_MOE: u32 = 1 << 15;

pub struct DShotDma {
    buffer1: [u32; DSHOT_DMA_BUFFER_SIZE],  // MOTOR1 (PC8, TIM3_CH3)
    buffer2: [u32; DSHOT_DMA_BUFFER_SIZE],  // MOTOR2 (PC9, TIM3_CH4)
    buffer3: [u32; DSHOT_DMA_BUFFER_SIZE],  // MOTOR3 (PA8, TIM1_CH1)
    buffer4: [u32; DSHOT_DMA_BUFFER_SIZE],  // MOTOR4 (PA9, TIM1_CH2)
    period: u16,
}

impl DShotDma {
    pub fn new() -> Self {
        Self {
            buffer1: [0; DSHOT_DMA_BUFFER_SIZE],
            buffer2: [0; DSHOT_DMA_BUFFER_SIZE],
            buffer3: [0; DSHOT_DMA_BUFFER_SIZE],
            buffer4: [0; DSHOT_DMA_BUFFER_SIZE],
            period: 19,
        }
    }

    pub fn init(&mut self) {
        // APB1 timer clock = 108 MHz (2x APB1 = 2x54MHz)
        // APB2 timer clock = 216 MHz (2x APB2 = 2x108MHz) - but TIM1 uses APB2
        // Actually for STM32F7 at 216MHz: APB1 timers = 108MHz, APB2 timers = 216MHz
        // We need same bit timing, so TIM1 needs different prescaler

        let tim3_clock = 108_000_000u32;  // APB1 timer clock
        let tim1_clock = 216_000_000u32;  // APB2 timer clock

        let tim3_prescaler = ((tim3_clock / (20 * 300_000)) - 1) as u16;  // = 17
        let tim1_prescaler = ((tim1_clock / (20 * 300_000)) - 1) as u16;  // = 35
        self.period = 19;

        unsafe {
            // Enable clocks: DMA1, DMA2, GPIOA, GPIOC
            let ahb1enr = core::ptr::read_volatile(RCC_AHB1ENR as *const u32);
            core::ptr::write_volatile(
                RCC_AHB1ENR as *mut u32,
                ahb1enr | (1 << 0) | (1 << 2) | (1 << 21) | (1 << 22), // GPIOA, GPIOC, DMA1, DMA2
            );

            // Enable TIM3 clock (APB1)
            let apb1enr = core::ptr::read_volatile(RCC_APB1ENR as *const u32);
            core::ptr::write_volatile(RCC_APB1ENR as *mut u32, apb1enr | (1 << 1));

            // Enable TIM1 clock (APB2)
            let apb2enr = core::ptr::read_volatile(RCC_APB2ENR as *const u32);
            core::ptr::write_volatile(RCC_APB2ENR as *mut u32, apb2enr | (1 << 0));

            // Configure GPIO PC8/PC9 as AF2 (TIM3_CH3/CH4)
            let moder = core::ptr::read_volatile((GPIOC_BASE + 0x00) as *const u32);
            core::ptr::write_volatile(
                (GPIOC_BASE + 0x00) as *mut u32,
                (moder & 0xFFF0_FFFF) | 0x000A_0000,
            );
            let afrh = core::ptr::read_volatile((GPIOC_BASE + 0x24) as *const u32);
            core::ptr::write_volatile(
                (GPIOC_BASE + 0x24) as *mut u32,
                (afrh & 0xFFFF_FF00) | 0x22,
            );
            let ospeedr = core::ptr::read_volatile((GPIOC_BASE + 0x08) as *const u32);
            core::ptr::write_volatile(
                (GPIOC_BASE + 0x08) as *mut u32,
                (ospeedr & 0xFFF0_FFFF) | 0x000F_0000,  // Very high speed
            );

            // Configure GPIO PA8/PA9 as AF1 (TIM1_CH1/CH2)
            let moder = core::ptr::read_volatile((GPIOA_BASE + 0x00) as *const u32);
            core::ptr::write_volatile(
                (GPIOA_BASE + 0x00) as *mut u32,
                (moder & 0xFFF0_FFFF) | 0x000A_0000,  // PA8/PA9 = AF mode
            );
            let afrh = core::ptr::read_volatile((GPIOA_BASE + 0x24) as *const u32);
            core::ptr::write_volatile(
                (GPIOA_BASE + 0x24) as *mut u32,
                (afrh & 0xFFFF_FF00) | 0x11,  // PA8=AF1, PA9=AF1
            );
            let ospeedr = core::ptr::read_volatile((GPIOA_BASE + 0x08) as *const u32);
            core::ptr::write_volatile(
                (GPIOA_BASE + 0x08) as *mut u32,
                (ospeedr & 0xFFF0_FFFF) | 0x000F_0000,  // Very high speed
            );

            // ===== Configure TIM3 (MOTOR1/2) =====
            core::ptr::write_volatile(TIM3_CR1 as *mut u32, 0);
            core::ptr::write_volatile(TIM3_PSC as *mut u32, tim3_prescaler as u32);
            core::ptr::write_volatile(TIM3_ARR as *mut u32, self.period as u32);
            core::ptr::write_volatile(TIM3_CCMR2 as *mut u32, 0x6868);  // PWM1 mode, preload CH3/CH4
            core::ptr::write_volatile(TIM3_CCER as *mut u32, 0x1100);   // Enable CH3/CH4 output
            core::ptr::write_volatile(TIM3_CCR3 as *mut u32, 0);
            core::ptr::write_volatile(TIM3_CCR4 as *mut u32, 0);
            core::ptr::write_volatile(TIM3_EGR as *mut u32, 0x1);       // Update event
            core::ptr::write_volatile(TIM3_DIER as *mut u32, 0x0);      // DMA disabled initially
            core::ptr::write_volatile(TIM3_CR1 as *mut u32, CR1_URS | CR1_CEN);

            // ===== Configure TIM1 (MOTOR3/4) =====
            core::ptr::write_volatile(TIM1_CR1 as *mut u32, 0);
            core::ptr::write_volatile(TIM1_PSC as *mut u32, tim1_prescaler as u32);
            core::ptr::write_volatile(TIM1_ARR as *mut u32, self.period as u32);
            core::ptr::write_volatile(TIM1_CCMR1 as *mut u32, 0x6868);  // PWM1 mode, preload CH1/CH2
            core::ptr::write_volatile(TIM1_CCER as *mut u32, 0x0011);   // Enable CH1/CH2 output
            core::ptr::write_volatile(TIM1_CCR1 as *mut u32, 0);
            core::ptr::write_volatile(TIM1_CCR2 as *mut u32, 0);
            core::ptr::write_volatile(TIM1_BDTR as *mut u32, BDTR_MOE); // Main output enable (required for TIM1)
            core::ptr::write_volatile(TIM1_EGR as *mut u32, 0x1);       // Update event
            core::ptr::write_volatile(TIM1_DIER as *mut u32, 0x0);      // DMA disabled initially
            core::ptr::write_volatile(TIM1_CR1 as *mut u32, CR1_URS | CR1_CEN);

            // ===== Configure DMA streams =====
            // Disable all DMA streams first
            self.disable_dma(DMA1_STREAM7_BASE);
            self.disable_dma(DMA1_STREAM2_BASE);
            self.disable_dma(DMA2_STREAM1_BASE);
            self.disable_dma(DMA2_STREAM2_BASE);

            // Clear all DMA flags
            core::ptr::write_volatile(DMA1_HIFCR as *mut u32, DMA1_ALL_FLAGS7);
            core::ptr::write_volatile(DMA1_LIFCR as *mut u32, DMA1_ALL_FLAGS2);
            core::ptr::write_volatile(DMA2_LIFCR as *mut u32, DMA2_ALL_FLAGS1 | DMA2_ALL_FLAGS2);

            // DMA1 Stream7 -> TIM3_CCR3 (Channel 5)
            let dma_cr_tim3 = CHSEL_CH5 | DMA_CR_MINC | DMA_CR_PSIZE_32 | DMA_CR_MSIZE_32 | DMA_CR_DIR_M2P;
            core::ptr::write_volatile((DMA1_STREAM7_BASE + DMA_SxCR) as *mut u32, dma_cr_tim3);
            core::ptr::write_volatile((DMA1_STREAM7_BASE + DMA_SxPAR) as *mut u32, TIM3_CCR3);
            core::ptr::write_volatile((DMA1_STREAM7_BASE + DMA_SxM0AR) as *mut u32, self.buffer1.as_ptr() as u32);

            // DMA1 Stream2 -> TIM3_CCR4 (Channel 5)
            core::ptr::write_volatile((DMA1_STREAM2_BASE + DMA_SxCR) as *mut u32, dma_cr_tim3);
            core::ptr::write_volatile((DMA1_STREAM2_BASE + DMA_SxPAR) as *mut u32, TIM3_CCR4);
            core::ptr::write_volatile((DMA1_STREAM2_BASE + DMA_SxM0AR) as *mut u32, self.buffer2.as_ptr() as u32);

            // DMA2 Stream1 -> TIM1_CCR1 (Channel 6)
            let dma_cr_tim1 = CHSEL_CH6 | DMA_CR_MINC | DMA_CR_PSIZE_32 | DMA_CR_MSIZE_32 | DMA_CR_DIR_M2P;
            core::ptr::write_volatile((DMA2_STREAM1_BASE + DMA_SxCR) as *mut u32, dma_cr_tim1);
            core::ptr::write_volatile((DMA2_STREAM1_BASE + DMA_SxPAR) as *mut u32, TIM1_CCR1);
            core::ptr::write_volatile((DMA2_STREAM1_BASE + DMA_SxM0AR) as *mut u32, self.buffer3.as_ptr() as u32);

            // DMA2 Stream2 -> TIM1_CCR2 (Channel 6)
            core::ptr::write_volatile((DMA2_STREAM2_BASE + DMA_SxCR) as *mut u32, dma_cr_tim1);
            core::ptr::write_volatile((DMA2_STREAM2_BASE + DMA_SxPAR) as *mut u32, TIM1_CCR2);
            core::ptr::write_volatile((DMA2_STREAM2_BASE + DMA_SxM0AR) as *mut u32, self.buffer4.as_ptr() as u32);
        }

        info!("DShot DMA initialized (4 channels: TIM3_CH3/4 + TIM1_CH1/2)");
    }

    fn disable_dma(&self, base: u32) {
        unsafe {
            core::ptr::write_volatile((base + DMA_SxCR) as *mut u32, 0);
            while core::ptr::read_volatile((base + DMA_SxCR) as *const u32) & DMA_CR_EN != 0 {}
        }
    }

    /// Send DShot frames to all 4 motors
    /// frame1/2 = MOTOR1/2 (TIM3), frame3/4 = MOTOR3/4 (TIM1)
    pub fn send_frame_all(&mut self, frame1: u16, frame2: u16, frame3: u16, frame4: u16) {
        // Fill all buffers
        for i in 0..16 {
            self.buffer1[i] = if (frame1 >> (15 - i)) & 1 == 1 { 14 } else { 7 };
            self.buffer2[i] = if (frame2 >> (15 - i)) & 1 == 1 { 14 } else { 7 };
            self.buffer3[i] = if (frame3 >> (15 - i)) & 1 == 1 { 14 } else { 7 };
            self.buffer4[i] = if (frame4 >> (15 - i)) & 1 == 1 { 14 } else { 7 };
        }
        // Trailing zeros for idle
        self.buffer1[16] = 0; self.buffer1[17] = 0;
        self.buffer2[16] = 0; self.buffer2[17] = 0;
        self.buffer3[16] = 0; self.buffer3[17] = 0;
        self.buffer4[16] = 0; self.buffer4[17] = 0;

        self.send_dma_frame_all();
    }

    /// Legacy: send to MOTOR1/2 only (also sends 0 to MOTOR3/4)
    pub fn send_frame(&mut self, frame1: u16, frame2: u16) {
        self.send_frame_all(frame1, frame2, 0, 0);
    }

    fn send_dma_frame_all(&mut self) {
        unsafe {
            // Wait for previous transfers to complete
            self.wait_dma_complete();

            // Disable all timer DMA requests
            core::ptr::write_volatile(TIM3_DIER as *mut u32, 0);
            core::ptr::write_volatile(TIM1_DIER as *mut u32, 0);

            // Disable and reset all DMA streams
            self.disable_dma(DMA1_STREAM7_BASE);
            self.disable_dma(DMA1_STREAM2_BASE);
            self.disable_dma(DMA2_STREAM1_BASE);
            self.disable_dma(DMA2_STREAM2_BASE);

            // Clear all DMA flags
            core::ptr::write_volatile(DMA1_HIFCR as *mut u32, DMA1_ALL_FLAGS7);
            core::ptr::write_volatile(DMA1_LIFCR as *mut u32, DMA1_ALL_FLAGS2);
            core::ptr::write_volatile(DMA2_LIFCR as *mut u32, DMA2_ALL_FLAGS1 | DMA2_ALL_FLAGS2);

            // Preload first bit to CCR registers
            core::ptr::write_volatile(TIM3_CCR3 as *mut u32, self.buffer1[0]);
            core::ptr::write_volatile(TIM3_CCR4 as *mut u32, self.buffer2[0]);
            core::ptr::write_volatile(TIM1_CCR1 as *mut u32, self.buffer3[0]);
            core::ptr::write_volatile(TIM1_CCR2 as *mut u32, self.buffer4[0]);

            // Configure DMA for remaining 17 values
            core::ptr::write_volatile((DMA1_STREAM7_BASE + DMA_SxNDTR) as *mut u32, 17);
            core::ptr::write_volatile((DMA1_STREAM7_BASE + DMA_SxM0AR) as *mut u32, self.buffer1.as_ptr().add(1) as u32);

            core::ptr::write_volatile((DMA1_STREAM2_BASE + DMA_SxNDTR) as *mut u32, 17);
            core::ptr::write_volatile((DMA1_STREAM2_BASE + DMA_SxM0AR) as *mut u32, self.buffer2.as_ptr().add(1) as u32);

            core::ptr::write_volatile((DMA2_STREAM1_BASE + DMA_SxNDTR) as *mut u32, 17);
            core::ptr::write_volatile((DMA2_STREAM1_BASE + DMA_SxM0AR) as *mut u32, self.buffer3.as_ptr().add(1) as u32);

            core::ptr::write_volatile((DMA2_STREAM2_BASE + DMA_SxNDTR) as *mut u32, 17);
            core::ptr::write_volatile((DMA2_STREAM2_BASE + DMA_SxM0AR) as *mut u32, self.buffer4.as_ptr().add(1) as u32);

            // Trigger update events to load CCR preload and reset counters
            core::ptr::write_volatile(TIM3_EGR as *mut u32, 0x1);
            core::ptr::write_volatile(TIM1_EGR as *mut u32, 0x1);

            // Enable DMA streams
            let cr_tim3 = CHSEL_CH5 | DMA_CR_MINC | DMA_CR_PSIZE_32 | DMA_CR_MSIZE_32 | DMA_CR_DIR_M2P | DMA_CR_EN;
            let cr_tim1 = CHSEL_CH6 | DMA_CR_MINC | DMA_CR_PSIZE_32 | DMA_CR_MSIZE_32 | DMA_CR_DIR_M2P | DMA_CR_EN;

            core::ptr::write_volatile((DMA1_STREAM7_BASE + DMA_SxCR) as *mut u32, cr_tim3);
            core::ptr::write_volatile((DMA1_STREAM2_BASE + DMA_SxCR) as *mut u32, cr_tim3);
            core::ptr::write_volatile((DMA2_STREAM1_BASE + DMA_SxCR) as *mut u32, cr_tim1);
            core::ptr::write_volatile((DMA2_STREAM2_BASE + DMA_SxCR) as *mut u32, cr_tim1);

            // Enable timer DMA requests
            // TIM3: CC3DE (bit 11) + CC4DE (bit 12)
            core::ptr::write_volatile(TIM3_DIER as *mut u32, (1 << 11) | (1 << 12));
            // TIM1: CC1DE (bit 9) + CC2DE (bit 10)
            core::ptr::write_volatile(TIM1_DIER as *mut u32, (1 << 9) | (1 << 10));
        }
    }

    fn wait_dma_complete(&self) {
        unsafe {
            // Check if any DMA is still active
            let cr1 = core::ptr::read_volatile((DMA1_STREAM7_BASE + DMA_SxCR) as *const u32);
            let cr2 = core::ptr::read_volatile((DMA1_STREAM2_BASE + DMA_SxCR) as *const u32);
            let cr3 = core::ptr::read_volatile((DMA2_STREAM1_BASE + DMA_SxCR) as *const u32);
            let cr4 = core::ptr::read_volatile((DMA2_STREAM2_BASE + DMA_SxCR) as *const u32);

            if (cr1 | cr2 | cr3 | cr4) & DMA_CR_EN != 0 {
                // Wait for all transfers to complete or error
                loop {
                    let dma1_hisr = core::ptr::read_volatile(DMA1_HISR as *const u32);
                    let dma1_lisr = core::ptr::read_volatile(DMA1_LISR as *const u32);
                    let dma2_lisr = core::ptr::read_volatile(DMA2_LISR as *const u32);

                    let tim3_done = (dma1_hisr & DMA1_TCIF7) != 0 && (dma1_lisr & DMA1_TCIF2) != 0;
                    let tim1_done = (dma2_lisr & DMA2_TCIF1) != 0 && (dma2_lisr & DMA2_TCIF2) != 0;
                    let any_error = (dma1_hisr & DMA1_TEIF7) != 0
                        || (dma1_lisr & DMA1_TEIF2) != 0
                        || (dma2_lisr & DMA2_TEIF1) != 0
                        || (dma2_lisr & DMA2_TEIF2) != 0;

                    if (tim3_done && tim1_done) || any_error {
                        break;
                    }
                }
            }
        }
    }
}

impl Default for DShotDma {
    fn default() -> Self {
        Self::new()
    }
}
