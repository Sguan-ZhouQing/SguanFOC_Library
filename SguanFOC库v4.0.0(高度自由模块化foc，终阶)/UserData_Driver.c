#include "UserData_Driver.h"



uint32_t driver_read_tick(void){
    // 此处放置您的tick函数
    return 0;
}

SguanQ driver_read_encoder(uint8_t motor){
    SguanQ encoder_raw = iqmath_zero();
    switch (motor){
    case MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_SIX:
        // 此处放置您的UserData函数
    
        break;
    
    default:
        break;
    }
    return encoder_raw;
}

uint8_t driver_read_hall(uint8_t motor, uint8_t ch){
    uint8_t hall_raw = 0;
    switch (motor){
    case MOTOR_ONE:
        // 此处放置您的UserData函数
        switch (ch){
        case HALL_A:

            break;
        case HALL_B:

            break;
        case HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_TWO:
        // 此处放置您的UserData函数
        switch (ch){
        case HALL_A:

            break;
        case HALL_B:

            break;
        case HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_THREE:
        // 此处放置您的UserData函数
        switch (ch){
        case HALL_A:

            break;
        case HALL_B:

            break;
        case HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_FOUR:
        // 此处放置您的UserData函数
        switch (ch){
        case HALL_A:

            break;
        case HALL_B:

            break;
        case HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_FIVE:
        // 此处放置您的UserData函数
        switch (ch){
        case HALL_A:

            break;
        case HALL_B:

            break;
        case HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_SIX:
        // 此处放置您的UserData函数
        switch (ch){
        case HALL_A:

            break;
        case HALL_B:

            break;
        case HALL_C:

            break;
        
        default:
            break;
        }
        break;
    
    default:
        break;
    }
}

uint16_t driver_read_iabc(uint8_t motor, uint8_t ch){
    uint16_t iabc_raw = 0;
    switch (motor){
    case MOTOR_ONE:
        // 此处放置您的UserData函数
        switch (ch){
        case CURRENT_U:

            break;
        case CURRENT_V:

            break;
        case CURRENT_W:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_TWO:
        // 此处放置您的UserData函数
        switch (ch){
        case CURRENT_U:

            break;
        case CURRENT_V:

            break;
        case CURRENT_W:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_THREE:
        // 此处放置您的UserData函数
        switch (ch){
        case CURRENT_U:

            break;
        case CURRENT_V:

            break;
        case CURRENT_W:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_FOUR:
        // 此处放置您的UserData函数
        switch (ch){
        case CURRENT_U:

            break;
        case CURRENT_V:

            break;
        case CURRENT_W:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_FIVE:
        // 此处放置您的UserData函数
        switch (ch){
        case CURRENT_U:

            break;
        case CURRENT_V:

            break;
        case CURRENT_W:

            break;
        
        default:
            break;
        }
        break;
    case MOTOR_SIX:
        // 此处放置您的UserData函数
        switch (ch){
        case CURRENT_U:

            break;
        case CURRENT_V:

            break;
        case CURRENT_W:

            break;
        
        default:
            break;
        }
        break;
    
    default:
        break;
    }
    return iabc_raw;
}

SguanQ driver_read_vbus(uint8_t motor){
    SguanQ vbus_raw = iqmath_zero();
    switch (motor){
    case MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_SIX:
        // 此处放置您的UserData函数
    
        break;
    
    default:
        break;
    }
    return vbus_raw;
}

SguanQ driver_read_ibus(uint8_t motor){
    SguanQ ibus_raw = iqmath_zero();
    switch (motor){
    case MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_SIX:
        // 此处放置您的UserData函数
    
        break;
    
    default:
        break;
    }
    return ibus_raw;
}

SguanQ driver_read_temp_motor(uint8_t motor){
    SguanQ temp_motor = iqmath_zero();
    switch (motor){
    case MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_SIX:
        // 此处放置您的UserData函数
    
        break;
    
    default:
        break;
    }
    return temp_motor;
}

SguanQ driver_read_temp_driver(uint8_t motor){
    SguanQ temp_driver = iqmath_zero();
    switch (motor){
    case MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_SIX:
        // 此处放置您的UserData函数
    
        break;
    
    default:
        break;
    }
    return temp_driver;
}

SguanQ driver_read_temp_pcb(uint8_t motor){
    SguanQ temp_pcb_raw = iqmath_zero();
    switch (motor){
    case MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case MOTOR_SIX:
        // 此处放置您的UserData函数
    
        break;
    
    default:
        break;
    }
    return temp_pcb_raw;
}

void driver_set_uart(uint8_t *ch, uint16_t size){
    
}

void driver_set_pwm(uint8_t motor, 
    uint16_t du_0, 
    uint16_t du_1, 
    uint16_t dv_0, 
    uint16_t dv_1, 
    uint16_t dw_0, 
    uint16_t dw_1){
    switch (motor){
    // 此处放置您的UserData函数
    case MOTOR_ONE:
    
        break;
    case MOTOR_TWO:
    
        break;
    case MOTOR_THREE:
    
        break;
    case MOTOR_FOUR:
    
        break;
    case MOTOR_FIVE:
    
        break;
    case MOTOR_SIX:
    
        break;
    
    default:
        break;
    }
}
void driver_set_adc(uint8_t motor, uint16_t)