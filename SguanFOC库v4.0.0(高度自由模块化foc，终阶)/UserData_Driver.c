#include "UserData_Driver.h"



void driver_initial_init(void){

}

void driver_start_init(void){

}

void driver_stop_init(void){

}

uint32_t driver_read_tick(void){
    // 此处放置您的tick函数
    return 0;
}

SguanQ driver_read_encoder(uint8_t motor){
    SguanQ encoder_raw = iqmath_zero();
    switch (motor){
    case __LOCAL_MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_SIX:
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
    case __LOCAL_MOTOR_ONE:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_HALL_A:

            break;
        case __LOCAL_HALL_B:

            break;
        case __LOCAL_HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_TWO:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_HALL_A:

            break;
        case __LOCAL_HALL_B:

            break;
        case __LOCAL_HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_THREE:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_HALL_A:

            break;
        case __LOCAL_HALL_B:

            break;
        case __LOCAL_HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_FOUR:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_HALL_A:

            break;
        case __LOCAL_HALL_B:

            break;
        case __LOCAL_HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_FIVE:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_HALL_A:

            break;
        case __LOCAL_HALL_B:

            break;
        case __LOCAL_HALL_C:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_SIX:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_HALL_A:

            break;
        case __LOCAL_HALL_B:

            break;
        case __LOCAL_HALL_C:

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
    case __LOCAL_MOTOR_ONE:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_CURRENT_U:

            break;
        case __LOCAL_CURRENT_V:

            break;
        case __LOCAL_CURRENT_W:

            break;
        case __LOCAL_CURRENT_OTHER:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_TWO:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_CURRENT_U:

            break;
        case __LOCAL_CURRENT_V:

            break;
        case __LOCAL_CURRENT_W:

            break;
        case __LOCAL_CURRENT_OTHER:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_THREE:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_CURRENT_U:

            break;
        case __LOCAL_CURRENT_V:

            break;
        case __LOCAL_CURRENT_W:

            break;
        case __LOCAL_CURRENT_OTHER:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_FOUR:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_CURRENT_U:

            break;
        case __LOCAL_CURRENT_V:

            break;
        case __LOCAL_CURRENT_W:

            break;
        case __LOCAL_CURRENT_OTHER:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_FIVE:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_CURRENT_U:

            break;
        case __LOCAL_CURRENT_V:

            break;
        case __LOCAL_CURRENT_W:

            break;
        case __LOCAL_CURRENT_OTHER:

            break;
        
        default:
            break;
        }
        break;
    case __LOCAL_MOTOR_SIX:
        // 此处放置您的UserData函数
        switch (ch){
        case __LOCAL_CURRENT_U:

            break;
        case __LOCAL_CURRENT_V:

            break;
        case __LOCAL_CURRENT_W:

            break;
        case __LOCAL_CURRENT_OTHER:

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
    case __LOCAL_MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_SIX:
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
    case __LOCAL_MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_SIX:
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
    case __LOCAL_MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_SIX:
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
    case __LOCAL_MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_SIX:
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
    case __LOCAL_MOTOR_ONE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_TWO:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_THREE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FOUR:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_FIVE:
        // 此处放置您的UserData函数
    
        break;
    case __LOCAL_MOTOR_SIX:
        // 此处放置您的UserData函数
    
        break;
    
    default:
        break;
    }
    return temp_pcb_raw;
}

void driver_read_adc(uint8_t motor, uint16_t x){

}

void driver_set_pwm(uint8_t motor, 
    uint16_t du_0, uint16_t du_1, 
    uint16_t dv_0, uint16_t dv_1, 
    uint16_t dw_0, uint16_t dw_1){
    switch (motor){
    // 此处放置您的UserData函数
    case __LOCAL_MOTOR_ONE:
    
        break;
    case __LOCAL_MOTOR_TWO:
    
        break;
    case __LOCAL_MOTOR_THREE:
    
        break;
    case __LOCAL_MOTOR_FOUR:
    
        break;
    case __LOCAL_MOTOR_FIVE:
    
        break;
    case __LOCAL_MOTOR_SIX:
    
        break;
    
    default:
        break;
    }
}

void driver_send_uart(uint8_t *ch, uint16_t size){
    
}
