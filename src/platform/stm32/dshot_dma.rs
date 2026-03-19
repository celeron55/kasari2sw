use log::info;

const DMA1_BASE: u32 = 0x4002_6000;

// DMA status and flag clear registers
const DMA_LISR: u32 = 0x4002_6000;
const DMA_HISR: u32 = 0x4002_6004;
const DMA_LIFCR: u32 = 0x4002_6008;
const DMA_HIFCR: u32 = 0x4002_600C;

// Stream7 flags (bits 22-27)
const TCIF7: u32 = 1 << 27;
const TEIF7: u32 = 1 << 25;

// Stream2 flags (bits 16-21)
const TCIF2: u32 = 1 << 21;
const TEIF2: u32 = 1 << 19;

const ALL_FLAGS7: u32 = 0x0FC00000;  // Clear all Stream7 flags
const ALL_FLAGS2: u32 = 0x003D0000;  // Clear all Stream2 flags

const DMA_SxCR: u32 = 0x00;
const DMA_SxNDTR: u32 = 0x04;
const DMA_SxPAR: u32 = 0x08;
const DMA_SxM0AR: u32 = 0x0C;
const DMA_SxFCR: u32 = 0x10;

const DMA1_STREAM7_BASE: u32 = DMA1_BASE + 0x0B8;
const DMA1_STREAM2_BASE: u32 = DMA1_BASE + 0x040;

const DMA_CR_EN: u32 = 1 << 0;
const DMA_CR_DIR_0: u32 = 1 << 6;
const DMA_CR_MINC: u32 = 1 << 10;
const DMA_CR_PSIZE_WORD: u32 = (1 << 13) | (1 << 12);
const DMA_CR_MSIZE_WORD: u32 = (1 << 15) | (1 << 14);

const CHSEL_CHANNEL_5: u32 = (1 << 25) | (1 << 27);

pub const DSHOT_DMA_BUFFER_SIZE: usize = 18;

const TIM3_BASE: u32 = 0x4000_0400;
const TIM3_CR1: u32 = TIM3_BASE + 0x00;
const TIM3_DIER: u32 = TIM3_BASE + 0x0C;
const TIM3_ARR: u32 = TIM3_BASE + 0x2C;
const TIM3_CCR3: u32 = TIM3_BASE + 0x3C;
const TIM3_CCR4: u32 = TIM3_BASE + 0x40;

const CR1_CEN: u32 = 1 << 0;
const CR1_URS: u32 = 1 << 2;

pub struct DShotDma {
    buffer1: [u32; DSHOT_DMA_BUFFER_SIZE],
    buffer2: [u32; DSHOT_DMA_BUFFER_SIZE],
    dma_stream7: u32,
    dma_stream2: u32,
    period: u16,
}

impl DShotDma {
    pub fn new() -> Self {
        Self {
            buffer1: [0; DSHOT_DMA_BUFFER_SIZE],
            buffer2: [0; DSHOT_DMA_BUFFER_SIZE],
            dma_stream7: DMA1_STREAM7_BASE,
            dma_stream2: DMA1_STREAM2_BASE,
            period: 19,
        }
    }

    pub fn init(&mut self) {
        let timer_clock = 108_000_000;
        let prescaler = ((timer_clock / (20 * 300_000)) - 1) as u16;
        self.period = 19;

        unsafe {
            // Enable clocks: DMA1, GPIOC, TIM3
            let rcc_ahb1enr = core::ptr::read_volatile(0x4002_3830 as *mut u32);
            core::ptr::write_volatile(0x4002_3830 as *mut u32, rcc_ahb1enr | 0x1 | (1 << 2));

            let rcc_apb1enr = core::ptr::read_volatile(0x4002_3840 as *mut u32);
            core::ptr::write_volatile(0x4002_3840 as *mut u32, rcc_apb1enr | (1 << 1));

            // Configure GPIO PC8/PC9 as AF2 (TIM3_CH3/CH4), high speed
            let moder = core::ptr::read_volatile(0x4002_0800 as *mut u32);
            core::ptr::write_volatile(0x4002_0800 as *mut u32, (moder & 0xFFF0_0000) | 0x000A_0000);

            let afrh = core::ptr::read_volatile(0x4002_0824 as *mut u32);
            core::ptr::write_volatile(0x4002_0824 as *mut u32, (afrh & 0xFFFF_FF00) | 0x22);

            let ospeedr = core::ptr::read_volatile(0x4002_0808 as *mut u32);
            core::ptr::write_volatile(0x4002_0808 as *mut u32, (ospeedr & 0xFFF0_0000) | 0xA_0000);

            // Configure TIM3: prescaler, period, PWM mode, output enable
            core::ptr::write_volatile(TIM3_CR1 as *mut u32, 0);
            core::ptr::write_volatile((TIM3_BASE + 0x28) as *mut u32, prescaler as u32);
            core::ptr::write_volatile(TIM3_ARR as *mut u32, self.period as u32);
            core::ptr::write_volatile((TIM3_BASE + 0x1C) as *mut u32, 0x6868u32); // CCMR2: PWM1 mode, preload enable
            core::ptr::write_volatile((TIM3_BASE + 0x20) as *mut u32, 0x1100u32); // CCER: enable CH3/CH4

            // Set initial CCR values high to avoid immediate compare match
            core::ptr::write_volatile(TIM3_CCR3 as *mut u32, 0xFFFF);
            core::ptr::write_volatile(TIM3_CCR4 as *mut u32, 0xFFFF);

            core::ptr::write_volatile((TIM3_BASE + 0x14) as *mut u32, 0x1u32); // EGR: trigger update event

            // DMA requests disabled until frame send (per-frame control)
            core::ptr::write_volatile(TIM3_DIER as *mut u32, 0x0u32);

            // Start timer
            core::ptr::write_volatile(TIM3_CR1 as *mut u32, CR1_URS | CR1_CEN);

            // Disable DMA streams
            self.disable_dma(self.dma_stream7);
            self.disable_dma(self.dma_stream2);

            // Clear all DMA flags
            core::ptr::write_volatile(DMA_HIFCR as *mut u32, ALL_FLAGS7);
            core::ptr::write_volatile(DMA_LIFCR as *mut u32, ALL_FLAGS2);

            // Configure DMA: channel 5, memory increment, 32-bit, memory-to-peripheral
            let dma_cr =
                CHSEL_CHANNEL_5 | DMA_CR_MINC | DMA_CR_PSIZE_WORD | DMA_CR_MSIZE_WORD | DMA_CR_DIR_0;

            core::ptr::write_volatile((self.dma_stream7 + DMA_SxCR) as *mut u32, dma_cr);
            core::ptr::write_volatile((self.dma_stream7 + DMA_SxPAR) as *mut u32, TIM3_CCR3);
            core::ptr::write_volatile(
                (self.dma_stream7 + DMA_SxM0AR) as *mut u32,
                self.buffer1.as_ptr() as u32,
            );
            core::ptr::write_volatile(
                (self.dma_stream7 + DMA_SxNDTR) as *mut u32,
                DSHOT_DMA_BUFFER_SIZE as u32,
            );

            core::ptr::write_volatile((self.dma_stream2 + DMA_SxCR) as *mut u32, dma_cr);
            core::ptr::write_volatile((self.dma_stream2 + DMA_SxPAR) as *mut u32, TIM3_CCR4);
            core::ptr::write_volatile(
                (self.dma_stream2 + DMA_SxM0AR) as *mut u32,
                self.buffer2.as_ptr() as u32,
            );
            core::ptr::write_volatile(
                (self.dma_stream2 + DMA_SxNDTR) as *mut u32,
                DSHOT_DMA_BUFFER_SIZE as u32,
            );
        }

        info!("DShot DMA initialized (DShot300, TIM3_CH3/CH4, DMA1_Stream2/7)");
    }

    fn disable_dma(&self, base: u32) {
        unsafe {
            core::ptr::write_volatile((base + DMA_SxCR) as *mut u32, 0);
            while core::ptr::read_volatile((base + DMA_SxCR) as *const u32) & DMA_CR_EN != 0 {}
            core::ptr::write_volatile((base + DMA_SxNDTR) as *mut u32, 0x3F);
            core::ptr::write_volatile((base + DMA_SxFCR) as *mut u32, 0x3F);
        }
    }

    pub fn send_frame(&mut self, frame1: u16, frame2: u16) {
        for i in 0..16 {
            let bit1 = (frame1 >> (15 - i)) & 1;
            let bit2 = (frame2 >> (15 - i)) & 1;
            self.buffer1[i] = if bit1 == 1 { 14 } else { 7 };
            self.buffer2[i] = if bit2 == 1 { 14 } else { 7 };
        }
        self.buffer1[16] = 0;
        self.buffer1[17] = 0;
        self.buffer2[16] = 0;
        self.buffer2[17] = 0;

        self.send_dma_frame();
    }

    fn send_dma_frame(&mut self) {
        unsafe {
            // Wait for previous transfer to complete
            let cr7 = core::ptr::read_volatile((self.dma_stream7 + DMA_SxCR) as *const u32);
            let cr2 = core::ptr::read_volatile((self.dma_stream2 + DMA_SxCR) as *const u32);
            if (cr7 & DMA_CR_EN) != 0 || (cr2 & DMA_CR_EN) != 0 {
                loop {
                    let hisr = core::ptr::read_volatile(DMA_HISR as *const u32);
                    let lisr = core::ptr::read_volatile(DMA_LISR as *const u32);
                    if (hisr & TCIF7) != 0 && (lisr & TCIF2) != 0 { break; }
                    if (hisr & TEIF7) != 0 || (lisr & TEIF2) != 0 { break; }
                }
            }

            // Disable timer DMA requests
            core::ptr::write_volatile(TIM3_DIER as *mut u32, 0x0u32);

            // Disable and reset DMA streams
            self.disable_dma(self.dma_stream7);
            self.disable_dma(self.dma_stream2);
            core::ptr::write_volatile(DMA_HIFCR as *mut u32, ALL_FLAGS7);
            core::ptr::write_volatile(DMA_LIFCR as *mut u32, ALL_FLAGS2);

            // Preload first bit manually (buffer[0]) since DMA transfers on compare match
            core::ptr::write_volatile(TIM3_CCR3 as *mut u32, self.buffer1[0]);
            core::ptr::write_volatile(TIM3_CCR4 as *mut u32, self.buffer2[0]);

            // Configure DMA to transfer remaining 17 values (buffer[1]..buffer[17])
            core::ptr::write_volatile((self.dma_stream7 + DMA_SxNDTR) as *mut u32, 17);
            core::ptr::write_volatile((self.dma_stream2 + DMA_SxNDTR) as *mut u32, 17);
            core::ptr::write_volatile(
                (self.dma_stream7 + DMA_SxM0AR) as *mut u32,
                self.buffer1.as_ptr().add(1) as u32,
            );
            core::ptr::write_volatile(
                (self.dma_stream2 + DMA_SxM0AR) as *mut u32,
                self.buffer2.as_ptr().add(1) as u32,
            );

            // Trigger update event: loads CCR preload and resets counter to 0
            core::ptr::write_volatile((TIM3_BASE + 0x14) as *mut u32, 0x1);

            // Enable DMA streams
            let cr = CHSEL_CHANNEL_5 | DMA_CR_MINC | DMA_CR_PSIZE_WORD | DMA_CR_MSIZE_WORD | DMA_CR_DIR_0 | DMA_CR_EN;
            core::ptr::write_volatile((self.dma_stream7 + DMA_SxCR) as *mut u32, cr);
            core::ptr::write_volatile((self.dma_stream2 + DMA_SxCR) as *mut u32, cr);

            // Enable timer DMA requests to start transfer
            core::ptr::write_volatile(TIM3_DIER as *mut u32, 0x1800u32);
        }
    }
}

impl Default for DShotDma {
    fn default() -> Self {
        Self::new()
    }
}
