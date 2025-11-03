#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>

/* Addresses */
#define LCD_7BIT  0x27
#define RTC_7BIT  0x68
#define BUZZER_PIN 5   // PA5
#define SERVO_PIN 8    // PA8 (TIM1_CH1)

/* ---------------- Delay ---------------- */
static void delay_ms(uint32_t ms){ for(uint32_t i=0;i<ms*8000;++i)__NOP(); }

/* ---------------- I2C Init ---------------- */
static void I2C1_Init(void){
    RCC->AHB1ENR |= (1<<1);  // GPIOB
    RCC->APB1ENR |= (1<<21); // I2C1
    GPIOB->MODER &= ~((3<<12)|(3<<14));
    GPIOB->MODER |=  ((2<<12)|(2<<14));
    GPIOB->OTYPER |= (1<<6)|(1<<7);
    GPIOB->OSPEEDR|= (3<<12)|(3<<14);
    GPIOB->PUPDR  |= (1<<12)|(1<<14);
    GPIOB->AFR[0] |= (4<<24)|(4<<28);
    I2C1->CR1|=(1<<15); I2C1->CR1&=~(1<<15);
    I2C1->CR2=16; I2C1->CCR=80; I2C1->TRISE=17;
    I2C1->CR1|=(1<<10)|(1<<0);
}

/* ---------------- I2C helpers ---------------- */
static void i2c_write_buffer(uint8_t addr7,const uint8_t *data,uint8_t len){
    I2C1->CR1|=(1<<8); while(!(I2C1->SR1&(1<<0)));
    I2C1->DR=(addr7<<1); while(!(I2C1->SR1&(1<<1)));
    (void)I2C1->SR1;(void)I2C1->SR2;
    for(uint8_t i=0;i<len;++i){while(!(I2C1->SR1&(1<<7)));I2C1->DR=data[i];while(!(I2C1->SR1&(1<<2)));}
    I2C1->CR1|=(1<<9);
}
static int ds3231_read_regs(uint8_t reg,uint8_t *buf,uint8_t len){
    if(!len)return 0;
    I2C1->CR1|=(1<<8);while(!(I2C1->SR1&(1<<0)));I2C1->DR=(RTC_7BIT<<1);
    while(!(I2C1->SR1&(1<<1)));(void)I2C1->SR1;(void)I2C1->SR2;
    while(!(I2C1->SR1&(1<<7)));I2C1->DR=reg;while(!(I2C1->SR1&(1<<2)));I2C1->CR1|=(1<<9);
    I2C1->CR1|=(1<<8);while(!(I2C1->SR1&(1<<0)));I2C1->DR=(RTC_7BIT<<1)|1;
    while(!(I2C1->SR1&(1<<1)));(void)I2C1->SR1;(void)I2C1->SR2;
    for(uint8_t i=0;i<len;++i){
        if(i==len-1)I2C1->CR1&=~(1<<10);else I2C1->CR1|=(1<<10);
        while(!(I2C1->SR1&(1<<6)));buf[i]=I2C1->DR; if(i==len-1)I2C1->CR1|=(1<<9);
    }I2C1->CR1|=(1<<10);return 1;
}

/* ---------------- LCD ---------------- */
static void lcd_send_nibbles(uint8_t value,uint8_t rs){
    uint8_t hi=value&0xF0,lo=(value<<4)&0xF0,m=rs?0x01:0x00;
    uint8_t tx[4]={hi|m|0x0C,hi|m|0x08,lo|m|0x0C,lo|m|0x08};
    i2c_write_buffer(LCD_7BIT,tx,4);delay_ms(1);
}
static void LCD_Cmd(uint8_t cmd){lcd_send_nibbles(cmd,0);if(cmd==0x01||cmd==0x02)delay_ms(2);}
static void LCD_Data(uint8_t d){lcd_send_nibbles(d,1);}
static void LCD_Init(void){delay_ms(50);LCD_Cmd(0x33);LCD_Cmd(0x32);LCD_Cmd(0x28);LCD_Cmd(0x0C);LCD_Cmd(0x06);LCD_Cmd(0x01);}
static void LCD_SetCursor(uint8_t r,uint8_t c){LCD_Cmd((r==0?0x80:0xC0)+c);}
static void LCD_PrintPad(const char*s){char tmp[17];uint8_t i=0;while(*s&&i<16)tmp[i++]=*s++;while(i<16)tmp[i++]=' ';for(i=0;i<16;i++)LCD_Data(tmp[i]);}

/* ---------------- RTC ---------------- */
static uint8_t bcd2dec(uint8_t b){return((b>>4)*10)+(b&0x0F);}
static uint8_t dec2bcd(uint8_t d){return((d/10)<<4)|(d%10);}
static void RTC_SetTime(uint8_t h,uint8_t m,uint8_t s){uint8_t b[4]={0,dec2bcd(s),dec2bcd(m),dec2bcd(h)};i2c_write_buffer(RTC_7BIT,b,4);}
static int RTC_ReadTime(uint8_t*h,uint8_t*m,uint8_t*s){uint8_t t[3];if(!ds3231_read_regs(0,t,3))return 0;*s=bcd2dec(t[0]&0x7F);*m=bcd2dec(t[1]&0x7F);*h=bcd2dec(t[2]&0x3F);return 1;}
static void RTC_SetFromBuildTime(void){int h,m,s;sscanf(_TIME_,"%d:%d:%d",&h,&m,&s);RTC_SetTime(h,m,s);}

/* ---------------- Buzzer ---------------- */
static void Buzzer_Init(void){RCC->AHB1ENR|=(1<<0);GPIOA->MODER&=~(3<<(BUZZER_PIN*2));GPIOA->MODER|=(1<<(BUZZER_PIN*2));}
static void Buzzer_On(void){GPIOA->ODR|=(1<<BUZZER_PIN);}
static void Buzzer_Off(void){GPIOA->ODR&=~(1<<BUZZER_PIN);}
static void Buzzer_Beep(uint8_t times,uint16_t on_ms,uint16_t off_ms){
    for(uint8_t i=0;i<times;i++){Buzzer_On();delay_ms(on_ms);Buzzer_Off();delay_ms(off_ms);}
}

/* ---------------- Servo ---------------- */
static void Servo_Init(void){
    RCC->AHB1ENR |= (1<<0);   // Enable GPIOA clock
    RCC->APB2ENR |= (1<<0);   // Enable TIM1 clock
    
    // Configure PA8 as alternate function (TIM1_CH1)
    GPIOA->MODER &= ~(3<<(SERVO_PIN*2));
    GPIOA->MODER |= (2<<(SERVO_PIN*2));
    GPIOA->OSPEEDR |= (3<<(SERVO_PIN*2));
    GPIOA->AFR[1] &= ~(0xF<<((SERVO_PIN-8)*4));
    GPIOA->AFR[1] |= (1<<((SERVO_PIN-8)*4));
    
    // For TIM1 on APB2 (typically higher frequency than APB1)
    // Assuming 16 MHz APB2 clock:
    TIM1->PSC = 16 - 1;        // Prescaler = 16-1 ? 1 MHz timer clock
    TIM1->ARR = 20000 - 1;     // 20ms period (50Hz for servo)
    TIM1->CCR1 = 1500;         // 1.5ms pulse (center position)
    
    TIM1->CCMR1 = (6<<4) | (1<<3);  // PWM mode 1, preload enable
    TIM1->CCER |= (1<<0);            // Enable CH1 output
    TIM1->BDTR |= (1<<15);           // Main output enable (required for TIM1)
    TIM1->EGR |= (1<<0);             // Update generation
    TIM1->CR1 |= (1<<0);             // Enable timer
}

static void Servo_SetAngle(uint8_t angle){
    // 1ms (1000) = 0°, 2ms (2000) = 180°
    uint16_t pulse = 1000 + (angle * 1000) / 180;
    TIM1->CCR1 = pulse;
}

/*static void Print_Clock_Info(void){
    uint32_t sysclk = SystemCoreClock;
    uint32_t apb1_freq = sysclk / 4;  // Usually SYSCLK/4 for APB1
    
    char buf[20];
    sprintf(buf, "SYS:%lu MHz", sysclk/1000000);
    LCD_SetCursor(0,0);
    LCD_PrintPad(buf);
    
    sprintf(buf, "APB1:%lu MHz", apb1_freq/1000000);
    LCD_SetCursor(1,0);
    LCD_PrintPad(buf);
    
    delay_ms(3000);
}*/
/* ---------------- MAIN ---------------- */
int main(void){
    char buf[20];
    uint8_t h,m,s,last=255;
    uint8_t msg_active=0,msg_start=0;
    const uint8_t MSG_DURATION=2,MSG_INTERVAL=15,REFILL_DELAY=3,REFILL_DURATION=3;

    uint8_t pill_count=0,refill_active=0,refill_start=0,refill_waiting=0,refill_wait_start=0;

    I2C1_Init();LCD_Init();Buzzer_Init();Servo_Init();
	  //Print_Clock_Info();
    LCD_SetCursor(0,0);LCD_PrintPad("SMART PILL DISPENSER");delay_ms(1000);LCD_Cmd(0x01);
    RTC_SetFromBuildTime();
    Servo_SetAngle(0);

    while(1){
        if(!RTC_ReadTime(&h,&m,&s))continue;
        if(s!=last){
            LCD_SetCursor(0,0);
            sprintf(buf,"TIME %02d:%02d:%02d",h,m,s);
            LCD_PrintPad(buf);

            /* Servo position based on pill count */
            uint8_t servo_angle = pill_count * 60;  // 0°, 60°, 120° for pills 0,1,2

            /* TAKE TABLET */
            if((s%MSG_INTERVAL)==0&&s!=last&&!refill_active){
                msg_active=1;msg_start=s;Buzzer_On();
                LCD_SetCursor(1,0);LCD_PrintPad("TAKE TABLET");
                
                // Move servo 60 degrees for each pill
                Servo_SetAngle(servo_angle + 60);
                pill_count++;
            }

            /* end of TAKE TABLET */
            if(msg_active){
                uint8_t elapsed=(s+60-msg_start)%60;
                if(elapsed>=MSG_DURATION){
                    msg_active=0;Buzzer_Off();
                    
                    // After 3rd pill, return servo to initial position (0°)
                    if(pill_count>=3){
                        Servo_SetAngle(0);
                        refill_waiting=1;refill_wait_start=s;
                    }
                    
                    LCD_SetCursor(1,0);LCD_PrintPad("                ");
                }
            }

            /* Wait 3 seconds before refill message */
            if(refill_waiting){
                uint8_t elapsed=(s+60-refill_wait_start)%60;
                if(elapsed>=REFILL_DELAY){
                    refill_waiting=0;refill_active=1;refill_start=s;
                    LCD_SetCursor(1,0);LCD_PrintPad("REFILL TABLETS");
                    /* play different buzzer pattern (3 quick beeps) */
                    Buzzer_Beep(3,100,100);
                }
            }

            /* REFILL message active for REFILL_DURATION seconds */
            if(refill_active){
                uint8_t elapsed=(s+60-refill_start)%60;
                if(elapsed>=REFILL_DURATION){
                    refill_active=0;pill_count=0;
                    LCD_SetCursor(1,0);LCD_PrintPad("                ");
                }
            }

            last=s;
        }
    }
}