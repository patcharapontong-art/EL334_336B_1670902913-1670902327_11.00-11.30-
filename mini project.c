#include "MK64F12.h"
 
/* Global Variables */
volatile int32_t countdown_sec = 10; 
volatile uint8_t system_state = 0;   /* 0 = Pause/Setup, 1 = Running, 2 = Alarm */
volatile uint8_t tick_beep = 0;
volatile uint8_t alarm_note_step = 0;
 
/* 7-Segment */
uint8_t border_step = 0;
uint16_t border_speed_cnt = 0;
 
/* 7-Segment Active-Low bit patterns (0-9) */
const uint8_t seg_code[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90
};
const uint8_t digit_select[4] = {0xF1, 0xF2, 0xF4, 0xF8};
 
/* Segment */
const uint8_t border_digit[12] = {
    0, 1, 2, 3,   
    3, 3,         
    3, 2, 1, 0,   
    0, 0          
};
 
const uint8_t border_pattern[12] = {
    0xFE, 0xFE, 0xFE, 0xFE,   /* Segment A 
    0xFD, 0xFB,               /* Segment B, C 
    0xF7, 0xF7, 0xF7, 0xF7,   /* Segment D 
    0xEF, 0xDF                /* Segment E, F 
};
 
/* Octave 6 (1,046 Hz - 2,093 Hz) */
#define TONE_C6   10020UL  /* ?? (1046.50 Hz) */
#define TONE_D6   8927UL   /* ?? (1174.66 Hz) */
#define TONE_E6   7953UL   /* ?? (1318.51 Hz) */
#define TONE_F6   7506UL   /* ?? (1396.91 Hz) */
#define TONE_G6   6688UL   /* ??? (1567.98 Hz) */
#define TONE_A6   5958UL   /* ?? (1760.00 Hz) */
#define TONE_Bb6  5624UL   /* ?????? (1864.66 Hz) */
#define TONE_C7   5010UL   /* ????? (2093.00 Hz) */
 
/* ????? "We Wish You a Merry Christmas and a Happy New Year" (30 ????) */
#define MELODY_LEN 30
const uint32_t melody_tone[MELODY_LEN] = {
    TONE_C6,                                         /* We */
    TONE_F6, TONE_F6, TONE_G6, TONE_F6, TONE_E6,     /* wish you a Mer-ry */
    TONE_D6, TONE_D6,                                /* Christ-mas */
    TONE_D6,                                         /* We */
    TONE_G6, TONE_G6, TONE_A6, TONE_G6, TONE_F6,     /* wish you a Mer-ry */
    TONE_E6, TONE_C6,                                /* Christ-mas */
    TONE_C6,                                         /* We */
    TONE_A6, TONE_A6, TONE_Bb6, TONE_A6, TONE_G6,    /* wish you a Mer-ry */
    TONE_F6, TONE_D6,                                /* Christ-mas */
    TONE_D6, TONE_D6,                                /* And a */
    TONE_D6, TONE_G6, TONE_E6,                       /* Hap-py New */
    TONE_F6                                          /* Year! */
};
 
/* (Staccato)  */
const uint32_t melody_ms[MELODY_LEN] = {
    280,                               /* We */
    280, 130, 130, 130, 130,           /* wish you a Mer-ry */
    280, 280,                          /* Christ-mas */
    280,                               /* We */
    280, 130, 130, 130, 130,           /* wish you a Mer-ry */
    280, 280,                          /* Christ-mas */
    280,                               /* We */
    280, 130, 130, 130, 130,           /* wish you a Mer-ry */
    280, 280,                          /* Christ-mas */
    130, 130,                          /* And a */
    280, 280, 280,                     /* Hap-py New */
    650                                /* Year! */
};
 
/* Function Prototypes */
void init_GPIO(void);
void init_PIT_Timer(void);
void rest_with_display_ms(uint32_t ms);
void PIT0_IRQHandler(void);
void PIT2_IRQHandler(void);
void PORTB_IRQHandler(void);
void shiftOut_595(uint8_t data);
void display_7seg(int32_t total_sec);
void display_7seg_border_chase(void);
void update_display(void);
void set_LED_bar(uint8_t level);
void update_VU_meter_by_tone(uint32_t tone_ticks);
void update_LED_status(void);
void play_tone_hw(uint32_t half_period_ticks, uint32_t duration_ms);
void check_alarm_melody(void);
 
/* --- 1. Hardware Initialization --- */
void init_GPIO(void) {
    SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | 
                   SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK);
 
    /* 4 LEDs on Shield (Active Low: 0 = ???, 1 = ???) */
    PORTD->PCR[1] = PORT_PCR_MUX(1); /* D13 (LED1) */
    PORTD->PCR[3] = PORT_PCR_MUX(1); /* D12 (LED2) */
    PORTD->PCR[2] = PORT_PCR_MUX(1); /* D11 (LED3) */
    PORTD->PCR[0] = PORT_PCR_MUX(1); /* D10 (LED4) */
 
    PTD->PDDR |= (1UL << 1) | (1UL << 3) | (1UL << 2) | (1UL << 0);
    PTD->PSOR |= (1UL << 1) | (1UL << 3) | (1UL << 2) | (1UL << 0);
 
    /* 74HC595 Shift Register Pins */
    PORTB->PCR[23] = PORT_PCR_MUX(1); /* D4 Latch */
    PORTC->PCR[3]  = PORT_PCR_MUX(1); /* D7 Clock */
    PORTC->PCR[12] = PORT_PCR_MUX(1); /* D8 Data */
    PTB->PDDR |= (1UL << 23);
    PTC->PDDR |= (1UL << 3) | (1UL << 12);
 
    /* Switch S1 (A1 -> PTB3), S2 (A2 -> PTB10), S3 (A3 -> PTB11) Interrupt */
    PORTB->PCR[3]  = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTB->PDDR &= ~((1UL << 3) | (1UL << 10) | (1UL << 11));
    NVIC_EnableIRQ(PORTB_IRQn);
}
 
void init_PIT_Timer(void) {
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0x00;
 
    /* Timer 0: 1 ?????????????????????? (Bus Clock 20.97152 MHz) */
    PIT->CHANNEL[0].LDVAL = 20971520UL - 1UL;
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
    NVIC_EnableIRQ(PIT0_IRQn);
 
    /* PIT2  */
    NVIC_SetPriority(PIT2_IRQn, 0);
    NVIC_EnableIRQ(PIT2_IRQn);
}
 
void rest_with_display_ms(uint32_t ms) {
    PIT->CHANNEL[1].TCTRL = 0;
    PIT->CHANNEL[1].LDVAL = (ms * 20972UL) - 1UL;
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
    PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TEN_MASK;
 
    while (!(PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK)) {
        update_display();
        if (system_state != 2) break;
    }
    PIT->CHANNEL[1].TCTRL = 0;
}
 
/* --- 2. Interrupt Service Routines (ISR) --- */
void PIT0_IRQHandler(void) {
    if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
 
        if (system_state == 1) {
            PTD->PTOR |= (1UL << 1);
            if (countdown_sec > 0) {
                countdown_sec--;
                tick_beep = 1;
            } else {
                system_state = 2;
                alarm_note_step = 0;
                border_step = 0;
            }
        }
    }
}
 
/* Interrupt 50% Duty Cycle  */
void PIT2_IRQHandler(void) {
    if (PIT->CHANNEL[2].TFLG & PIT_TFLG_TIF_MASK) {
        PIT->CHANNEL[2].TFLG = PIT_TFLG_TIF_MASK;
        PTA->PTOR |= (1UL << 1);
    }
}
 
void PORTB_IRQHandler(void) {
    if (PORTB->ISFR & (1UL << 3)) {
        PORTB->ISFR |= (1UL << 3);
        if (system_state == 0 && countdown_sec > 0) {
            system_state = 1;
            tick_beep = 1;
        } else if (system_state == 1) {
            system_state = 0;
        } else if (system_state == 2) {
            system_state = 0;
            alarm_note_step = 0;
            border_step = 0;
        }
    }
 
    if (PORTB->ISFR & (1UL << 10)) {
        PORTB->ISFR |= (1UL << 10);
        if (system_state == 0) {
            countdown_sec += 10;
            if (countdown_sec > 5999) countdown_sec = 10;
        }
    }
 
    if (PORTB->ISFR & (1UL << 11)) {
        PORTB->ISFR |= (1UL << 11);
        if (system_state == 0) {
            if (countdown_sec >= 10) {
                countdown_sec -= 10;
            } else {
                countdown_sec = 0;
            }
        }
    }
}
 
/* --- 3. Sub-functions --- */
void shiftOut_595(uint8_t data) {
    int i;
    volatile int d;
    for (i = 7; i >= 0; i--) {
        if (data & (1U << i)) PTC->PSOR |= (1UL << 12);
        else PTC->PCOR |= (1UL << 12);
 
        PTC->PSOR |= (1UL << 3);
        for (d = 0; d < 10; d++);
        PTC->PCOR |= (1UL << 3);
    }
}
 
void display_7seg(int32_t total_sec) {
    uint8_t mins = (uint8_t)(total_sec / 60);
    uint8_t secs = (uint8_t)(total_sec % 60);
    uint8_t digits[4];
    uint8_t pattern;
    int i;
    volatile int d;
 
    digits[0] = (mins / 10) % 10;
    digits[1] = mins % 10;
    digits[2] = (secs / 10) % 10;
    digits[3] = secs % 10;
 
    for (i = 0; i < 4; i++) {
        pattern = seg_code[digits[i]];
        if (i == 1) pattern &= 0x7F;
 
        PTB->PCOR |= (1UL << 23);
        shiftOut_595(pattern);
        shiftOut_595(digit_select[i]);
        PTB->PSOR |= (1UL << 23);
        for (d = 0; d < 600; d++); 
    }
}
 
void display_7seg_border_chase(void) {
    int i;
    volatile int d;
    uint8_t active_digit = border_digit[border_step];
    uint8_t active_seg   = border_pattern[border_step];
 
    for (i = 0; i < 4; i++) {
        PTB->PCOR |= (1UL << 23);
        if (i == active_digit) {
            shiftOut_595(active_seg);
        } else {
            shiftOut_595(0xFF);
        }
        shiftOut_595(digit_select[i]);
        PTB->PSOR |= (1UL << 23);
        for (d = 0; d < 600; d++); 
    }
 
    border_speed_cnt++;
    if (border_speed_cnt >= 28) { 
        border_speed_cnt = 0;
        border_step++;
        if (border_step >= 12) border_step = 0;
    }
}
 
void update_display(void) {
    if (system_state == 2) {
        display_7seg_border_chase();
    } else {
        display_7seg(countdown_sec);
    }
}
 
void set_LED_bar(uint8_t level) {
    PTD->PSOR |= (1UL << 1) | (1UL << 3) | (1UL << 2) | (1UL << 0);
 
    if (level >= 1) PTD->PCOR |= (1UL << 1);
    if (level >= 2) PTD->PCOR |= (1UL << 3);
    if (level >= 3) PTD->PCOR |= (1UL << 2);
    if (level >= 4) PTD->PCOR |= (1UL << 0);
}
 
void update_VU_meter_by_tone(uint32_t tone_ticks) {
    if (tone_ticks <= TONE_A6) {
        set_LED_bar(4);
    } else if (tone_ticks <= TONE_F6) {
        set_LED_bar(3);
    } else if (tone_ticks <= TONE_D6) {
        set_LED_bar(2);
    } else {
        set_LED_bar(1);
    }
}
 
void update_LED_status(void) {
    if (system_state == 0) {
        PTD->PSOR |= (1UL << 1) | (1UL << 3) | (1UL << 0);
        PTD->PCOR |= (1UL << 2);
    } else if (system_state == 1) {
        PTD->PCOR |= (1UL << 3);
        PTD->PSOR |= (1UL << 2) | (1UL << 0);
    }
}
 
void play_tone_hw(uint32_t half_period_ticks, uint32_t duration_ms) {
    PIT->CHANNEL[1].TCTRL = 0;
    PIT->CHANNEL[1].LDVAL = (duration_ms * 20972UL) - 1UL;
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
 
    PIT->CHANNEL[2].TCTRL = 0;
    PIT->CHANNEL[2].LDVAL = half_period_ticks - 1UL;
    PIT->CHANNEL[2].TFLG = PIT_TFLG_TIF_MASK;
 
    PORTA->PCR[1] = PORT_PCR_MUX(1);
    PTA->PDDR |= (1UL << 1);
 
    PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TEN_MASK;
    PIT->CHANNEL[2].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
 
    while (!(PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK)) {
        if (system_state == 2) {
            display_7seg_border_chase();
        }
        if (system_state == 0 && !tick_beep) {
            break;
        }
    }
 
    PIT->CHANNEL[1].TCTRL = 0;
    PIT->CHANNEL[2].TCTRL = 0;
    PTA->PSOR |= (1UL << 1);
    PORTA->PCR[1] = PORT_PCR_MUX(7) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
}
 
void check_alarm_melody(void) {
    if (tick_beep) {
        play_tone_hw(TONE_C7, 20);
        tick_beep = 0;
    }
 
    if (system_state == 2) {
        if (alarm_note_step < MELODY_LEN) {
            update_VU_meter_by_tone(melody_tone[alarm_note_step]);
            play_tone_hw(melody_tone[alarm_note_step], melody_ms[alarm_note_step]);
 
            /* 85ms */
            set_LED_bar(0);
            if (system_state == 2) {
                rest_with_display_ms(85);
            }
 
            alarm_note_step++;
        } else {
            set_LED_bar(0);
            rest_with_display_ms(700);
            alarm_note_step = 0;
        }
    }
}
 
/* --- 4. Main Function --- */
int main(void) {
    init_GPIO();
    init_PIT_Timer();
 
    while (1) {
        update_LED_status();
        check_alarm_melody();
        update_display();
    }
}