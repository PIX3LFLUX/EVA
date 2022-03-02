#include "serial_handling.h"

DRAM_ATTR Ticker no_activity_timer;                                 /* if no laps have been made for TIMEOUT_SECONDS, the CU is kept awake using the keep_cu_awake() function. */
DRAM_ATTR bool no_activity_timer_running = false;
HardwareSerial serial_interface_2(2);                               /* Serial interface with RX at pin 16 and TX at pin 17 */

DRAM_ATTR uint8_t receive_buffer[RECEIVE_BUFFER_LENGTH] = { 0 };    /* maximum received data in use should be 18 characters */
DRAM_ATTR uint8_t light_state = '0';                                /* State of the state machine in the control unit. 8 and 9 means early start, 1 is all LEDS on and 2...7 are the countdown. 0 is idle. */

DRAM_ATTR uint8_t race_status   = NO_RACE_GOING;                    /* for states, look in globals.h */
DRAM_ATTR uint8_t winning_car   = 0;

/* Car data for cars 0...3 */
DRAM_ATTR uint8_t  car_position               [4] = { 0 };
DRAM_ATTR uint8_t  car_laps                   [4] = { 0 };    /* total number of laps ran by the car */
DRAM_ATTR uint32_t car_timestamp              [4] = { 0 };    /* save counter state from Control Unit for each car */
DRAM_ATTR uint32_t car_timestamp_previous     [4] = { 0 };    /* required to calculate lap times */
DRAM_ATTR uint32_t car_lap_time               [4] = { 0 };
DRAM_ATTR uint32_t car_lap_time_previous      [4] = { 0 };

/* Statistics */
DRAM_ATTR  int64_t car_lap_time_improvement   [4] = { 0 };    /* to see how your lap times have improved */
DRAM_ATTR double_t car_lap_time_sum           [4] = { 0 };    /* divide this by the number of laps minus one to get the average lap time (first round does not have speed data and would not be representative since it's not a flying lap) */
DRAM_ATTR double_t car_lap_time_array    [256][4] = { 0 };    /* holds all lap times for all cars. Used to calculate standard deviation. */
DRAM_ATTR double_t winner_lap_time_average        = 0;        /* the winner's average lap time. */
DRAM_ATTR double_t winner_lap_time_standard       = 0;        /* the winner's standard deviation of lap times. */

void init_serial2()
{
    serial_interface_2.begin(19200);
    serial_interface_2.setTimeout(10);
    #if SERIAL_USERDATA_PRINT
        print_eva_logo();
    #endif
    xSemaphoreGive(serial2_access_semaphore);

    no_activity_timer.attach(TIMEOUT_SECONDS, keep_cu_awake);
    no_activity_timer_running = true;
}

inline void print_eva_logo()
{
    Serial.print(TEXT_PUSH_DOWN_STRING);
    Serial.printf("\r\n\
                     (=)\r\n\
 ______  __      __ (=/@/=)\r\n\
|  ____| \\ \\    / /  /\\(=)\r\n\
| |__     \\ \\  / /  /  \\\r\n\
|  __|     \\ \\/ /  / /\\ \\\r\n\
| |____     \\  /  / ____ \\\r\n\
|______|     \\/  /_/    \\_\\ \r\n");
}

IRAM_ATTR void get_data_from_control_unit()
{
    if ( xSemaphoreTake(serial2_access_semaphore,0) == pdTRUE )
    {    
        uint8_t buffer_index = 0;
        memset(receive_buffer, 0, RECEIVE_BUFFER_LENGTH); /* Clear receive buffer */
        
        while ( serial_interface_2.available() > 0 ) { serial_interface_2.read(); } /* Clear serial buffer */
        serial_interface_2.print(REQUEST_LAST_PASSING_TIMESTAMP);                   /* printing to the serial interface is equivalent to transmitting data */
        DELAY_N_MS(20);                                                             /* wait for response from CU. If the CU is polled too fast, it will never respond*/         

        while ( serial_interface_2.available() > 0 )
        {
            /* reading from the serial interface is equivalent to receiving data */
            uint8_t received_byte = serial_interface_2.read();
            receive_buffer[buffer_index] = received_byte;
            
            /* '$' is the terminating character of the messages the control unit sends. */
            if ( ( received_byte == '$' ) | ( buffer_index >= ( RECEIVE_BUFFER_LENGTH - 1 ) ) ) { break; }
            else                                                                                { buffer_index += 1; }
        }
        xSemaphoreGive(serial2_access_semaphore);
        parse_data_received();
    }
}

inline void parse_data_received()
{
    /* If a threshold is reached, the microcontroller will reboot because the control unit is likely turned off */
    static uint8_t empty_buffer_counter; 
    #if DEBUG
        //Serial.printf("Read buffer contents: %s\n", receive_buffer);
    #endif
    /* check for ':' (0x3A) as second received character. If it exists in the message, it does not contain counter data, but some other stats. See http://slotbaer.de/carrera-digital-124-132/10-cu-rundenzaehler-protokoll.html */
    if (receive_buffer[1] == ':')
    {
        /*
        No car passed the finish line recently.
        If the light state has changed since last time, we need to process that to synchronize the race state
        */        
        empty_buffer_counter = 0;
        /* if the no activity timer is not running, run it. */
        if ( !no_activity_timer_running )
        {
            no_activity_timer.attach(TIMEOUT_SECONDS, keep_cu_awake);
            no_activity_timer_running = true;
        } 

        /* only change light state variable and give semaphore when the light state changed*/
        if (light_state != receive_buffer[10])
        {
            light_state = receive_buffer[10];
            xSemaphoreGive(process_light_state_semaphore);
        }
    } 
    else if (receive_buffer[1] == 0)
    {
        empty_buffer_counter += 1;
        if (empty_buffer_counter > 10)
        {
            #if DEBUG
                Serial.println("Serial receive buffer is empty. Control unit is likely off. Rebooting...");
                delay(1000);
                ESP.restart();
            #endif
            #if SERIAL_USERDATA_PRINT
                Serial.println("Kann die Carrera Control Unit nicht finden.\r\nBitte die Control Unit ausschalten, mindestens zwei Sekunden warten und dann wieder einschalten.\r\nEVA startet gleich neu.");
                delay(10000);
                ESP.restart();
            #endif
        }
    }
    else
    {
        /* A car has passed the finish line recently. */

        /* disable the no activity timer */
        if ( no_activity_timer_running )
        {
            no_activity_timer.detach();
            no_activity_timer_running = false;
        } 

        /* DECODING OF THE TIMESTAMP
        The first character of the buffer is a '?', acknowledgement that the carrera control unit (CU) received the command.

        The second character is a number of 1...4, given in ASCII-Code. If 49 is substracted, the car number of 0...3 can be obtained as an integer.
        Port 0 is the leftmost port of the CU, port 3 is the rightmost port.

        The timestamp is containted in eight characters of the receive_buffer: receive_buffer[2 to 9], including.
        Carrera transmits one nibble of data per transmitted character, the lower one.
        The upper one is always 0x3 for the timestamp, this is likely to only get printable characters as output.
        The eight transmitted nibbles combine to make four bytes.
        In each of the four bytes, the lower nibble is transmitted first.
        However, the lowest byte is transmitted last.

        The last three characters are the group character, the checksum, and a '$' character. Those are not relevant for the timestamp.

        An example message (all numbers in HEX, 'enquoted' are ASCII characters):
        3F      31      30      30      30      30      33      3A      31      30      31      3D      24
        '?'     '1'      0       0       0       0       3       A       1       0      Group   CSum    '$'
        Ack     Car0        00              00              A3              01

        So the message contains the acknowledgement from the CU, and tells us that Car0 has passed the finish line at the time of 0x00 00 A3 01 (uint32_t 41729) milliseconds since the CU was powered up.
        Once this timestamp has been requested by the microcontroller, it can't be requested again.
        Instead, every time a car passes, a new timestamp becomes available.
        The lap time is obtained by substracting the last time stamp for the car from the current one.
        */ 
        uint8_t car_number = receive_buffer[1] - 49;
        /* car numbers can only be 0...3. If they are not, the CU is likely powered off. */
        if (car_number > 3) { return; }
        car_timestamp[car_number] = ((receive_buffer[3] - 0x30) << 28) +
                                    ((receive_buffer[2] - 0x30) << 24) +
                                    ((receive_buffer[5] - 0x30) << 20) +
                                    ((receive_buffer[4] - 0x30) << 16) +
                                    ((receive_buffer[7] - 0x30) << 12) +
                                    ((receive_buffer[6] - 0x30) << 8 ) +
                                    ((receive_buffer[9] - 0x30) << 4 ) +
                                     (receive_buffer[8] - 0x30);
        
        /* only calculate lap time if there is a previous, non-zero timestamp */
        if ( car_timestamp_previous[car_number] != 0 ) { car_lap_time[car_number] = car_timestamp[car_number] - car_timestamp_previous[car_number]; }
        car_timestamp_previous[car_number] = car_timestamp[car_number];

        if ( car_lap_time_previous[car_number] != 0 ) {  car_lap_time_improvement[car_number] = (int64_t)car_lap_time[car_number] - (int64_t)car_lap_time_previous[car_number]; }
        car_lap_time_previous[car_number] = car_lap_time[car_number];

        /* update buffers for statistics data */
        car_lap_time_sum[car_number]        += car_lap_time[car_number];
        car_lap_time_array[car_laps[car_number]][car_number] = car_lap_time[car_number];

        car_laps[car_number] += 1;

        /* sensorcar needs to be car 0 */
        if (car_number == 0)
        {
            send_data_wirelessly(CAR_NO_0_PASSED_FINISH_LINE); /* notify sensorcar that it passed the finish line */
        }

        /* if race is going (race_status is RACE_GOING) and a car has completed the required amount of laps, stop the race. */
        if ((race_status == RACE_GOING) && (car_laps[car_number] >= number_laps_in_race))
        {
            winning_car = car_number + 1;
            if (number_laps_in_race > 1) /* avoid divide by zero */
            {
                winner_lap_time_average     = car_lap_time_sum[car_number] / (number_laps_in_race - 1);
                double_t sum_buffer_squares = 0.0;
                for (uint8_t ii=1; ii<number_laps_in_race; ii++)
                {
                    sum_buffer_squares += (car_lap_time_array[ii][car_number] - winner_lap_time_average) * (car_lap_time_array[ii][car_number] - winner_lap_time_average);
                }
                winner_lap_time_standard    = sqrt( sum_buffer_squares / (number_laps_in_race - 1) );
            }
            xSemaphoreTake(dac_access_semaphore,0); /* lock DAC access */
            dac_output_voltage(DAC_CHANNEL_1, 0);   /* stop car */
            race_status = NO_RACE_GOING;
            send_data_wirelessly(race_status);
        }

        #if DEBUG
            Serial.printf("New timestamp detected: %dms\n", car_timestamp[car_number]);
            Serial.printf("Car Number %d has a lap time of %dms. \n", car_number, car_lap_time[car_number]);
        #endif

        xSemaphoreGive(print_data_semaphore);
    }
}

IRAM_ATTR void press_start_button()
{
    if ( xSemaphoreTake(serial2_access_semaphore,portMAX_DELAY) == pdTRUE )
    {   
        while (serial_interface_2.available() > 0) { serial_interface_2.read(); }   /* clear serial buffer */
        serial_interface_2.print(PRESS_START);                                      /* send request to start race */
        DELAY_N_MS(20);                                                             /* wait for response */
        while (serial_interface_2.available() > 0) { serial_interface_2.read(); }
        xSemaphoreGive(serial2_access_semaphore);
    }
}

/* Keeps the CU from going into sleep mode, which it does if nothing is done for 20 minutes. */
IRAM_ATTR void keep_cu_awake()
{
    dac_output_voltage(DAC_CHANNEL_1, 0);
    DELAY_N_MS(80);
    dac_output_voltage(DAC_CHANNEL_1, 15);
    DELAY_N_MS(80);
    dac_output_voltage(DAC_CHANNEL_1, 0);
}

IRAM_ATTR void print_car_data()
{
    /* No race is currently going but a car recently passed the finish line -> race just finished */
    if (race_status == NO_RACE_GOING)
    {
        /* Declare victor */
        #if DEBUG
            Serial.printf("Winner of the most recent race is car number %d.\n", winning_car);
        #endif
        #if SERIAL_USERDATA_PRINT
            print_victory_screen();
        #endif
        #if DISPLAY_OUTPUT_ENABLE
            display_victory_screen();
        #endif
        return;
    }
    /* A race is currently going and a car recently passed the finish line -> print statistics */
    else if (race_status == RACE_GOING)
    {
        /* Print header */
        #if DEBUG
            Serial.println("Position\tCar Number\tLaps\tLap Time\tImprovement");
        #endif
        #if SERIAL_USERDATA_PRINT
            Serial.print(TEXT_PUSH_DOWN_STRING); /* push old data down */
            Serial.printf("Position\tAuto\tRunden\tRundenzeit\tvs. Vorher\r\n");
        #endif
        #if DISPLAY_OUTPUT_ENABLE
            display_128x64.setFont(u8g2_font_ncenB08_tr);
            display_128x64.drawStr(0, 10,"POS AUTO RUND ZEIT");
            display_128x64.drawLine(0,11,128,11);
        #endif

        /* Weird for-loop, but its size is known at compile time. Additionally, operations to get the right element indices are only done once in the loop which makes it more efficient. */
        for (uint8_t number_cars = 4; number_cars > 0; number_cars--)
        {
            uint8_t ii = 4 - number_cars; 
            /* If the selected car has a timestamp other than 0, which is the initial value, print it. */
            if (car_timestamp[ii])
            {
                /* CAR POSITION ALGORITHM
                Hardcoded to avoid loops or heavy vector libraries. It subtracts booleans from a constant value.
                For example, if a car has more laps than itself and three other cars, it is in 5-(3+1)=1 st place.
                If it only has more laps than itself, it is in 5-(0+1)=4 th place.
                Cars that don't partake in the race have a lap value of 0.
                */
                car_position[ii] = 5 - 
                    int8_t(car_laps[ii] >= car_laps[0]) -
                    int8_t(car_laps[ii] >= car_laps[1]) -
                    int8_t(car_laps[ii] >= car_laps[2]) -
                    int8_t(car_laps[ii] >= car_laps[3]);
                #if DEBUG
                    Serial.printf("%d.\t\t#%d\t%d/%d\t%dms\t\t%lldms\n",
                        car_position[ii],
                        ii + 1,
                        car_laps[ii], number_laps_in_race,
                        car_lap_time[ii],
                        car_lap_time_improvement[ii]);
                #endif
                #if SERIAL_USERDATA_PRINT
                    Serial.printf("%d.\t\t#%d\t%d/%d\t%.3fs\t\t%.3fs\r\n",
                        car_position[ii],
                        ii + 1,
                        car_laps[ii], number_laps_in_race,
                        car_lap_time[ii]/1000.0,
                        car_lap_time_improvement[ii]/1000.0);
                #endif                
                #if DISPLAY_OUTPUT_ENABLE
                    display_128x64.setCursor(0, 12*(ii+2));
                    display_128x64.printf("%d. #%d %d/%d %dms",
                        car_position[ii],
                        ii + 1,
                        car_laps[ii], number_laps_in_race,
                        car_lap_time[ii]);
                #endif
            }
        }
        #if DISPLAY_OUTPUT_ENABLE
            display_128x64.sendBuffer();
            display_128x64.clearBuffer();
        #endif
    }
}

inline void display_victory_screen()
{
    display_128x64.setFont(u8g2_font_ncenB10_tr);
    display_128x64.drawXBM(0,0,finishflag_width,finishflag_height, finishflag_bits);
    display_128x64.drawStr(75,20,"Sieger:");
    display_128x64.setFont(u8g2_font_ncenB24_tr);
    display_128x64.setCursor(75,60);
    display_128x64.printf("#%d", winning_car);
    display_128x64.sendBuffer();
    display_128x64.clearBuffer();
}

inline void print_victory_screen()
{
    Serial.print(TEXT_PUSH_DOWN_STRING);
    Serial.printf("\
  ___________________\r\n\
||@|@| | |@|@| | |@|@|\r\n\
||@|@|_|_|@|@|_|_|@|@|\r\n\
|| | |@|@| | |@|@| | |\r\n\
||_|_|@|@|_|_|@|@|_|_|\r\n\
||@|@| | |@|@| | |@|@|\r\n\
||@|@|_|_|@|@|_|_|@|@|\r\n\
|| | |@|@| | |@|@| | |\r\n\
||_|_|@|@|_|_|@|@|_|_|\r\n\
||\r\n\
|| Sieger des Rennens:\r\n\
||    Spieler %d\r\n\
|| Mittlere Rundenzeit: %.3lfs\r\n\
|| Standardabweichung: %.3lfs\r\n", winning_car, winner_lap_time_average/1000.0, winner_lap_time_standard/1000.0);

}

IRAM_ATTR void process_light_state()
{
    /* The lighting state just changed. The new lighting state will be processed. */
    switch (light_state)
    {
        case '7':   /* countdown state 0 */
            #if SERIAL_USERDATA_PRINT
                Serial.println("LOS!");
            #endif
            race_status = RACE_GOING;
        break;        
        case '0':   /* this will happen after the countdown finished, or the CU is idling (middle LED on). Since this is not a clear indicator of a race, race_status will be untouched here. */
        break;
        case '1':   /* this is the ready or pause state. (all LEDs on) */
            /* reset race statistics. */
            memset(car_position,                0, 4);
            memset(car_laps,                    0, 4);
            memset(car_timestamp,               0, 16);
            memset(car_timestamp_previous,      0, 16);
            memset(car_lap_time,                0, 16);
            memset(car_lap_time_previous,       0, 16);
            memset(car_lap_time_improvement,    0, 32);
            memset(car_lap_time_sum,            0, 32); /* double is 8 byte on ESP32 */
            memset(car_lap_time_array,          0, 8192);
            race_status = NO_RACE_GOING;
            #if SERIAL_USERDATA_PRINT
                print_eva_logo();
                Serial.println("Der Countdown ist bereit.");
            #endif
        break;
        case '2':   /* countdown state 5. A new race will start soon. */
            #if SERIAL_USERDATA_PRINT
                Serial.println("Das Rennen startet gleich!");
            #endif
            race_status = NO_RACE_GOING;
        break;
        case '3':   /* countdown state 4 */
            #if SERIAL_USERDATA_PRINT
                Serial.println("AUF DIE PLAETZE...");
                break;
            #endif
            race_status = NO_RACE_GOING;
        break;            
        case '4':   /* countdown state 3 */
        break;    
        case '5':   /* countdown state 2 */
            #if SERIAL_USERDATA_PRINT
                Serial.println("FERTIG...");
            #endif
            race_status = NO_RACE_GOING;
        break;            
        case '6':   /* countdown state 1 */
        break;     
        case '8':   /* early start state 0, fallthrough on purpose */
            #if SERIAL_USERDATA_PRINT
                Serial.println("Fruehstart!");         
            #endif
        case '9':   /* early start state 1 */
            race_status = NO_RACE_GOING;
        break;
        default:
            #if DEBUG
                Serial.printf("Unknown light state: %c\n", light_state);
            #endif
        break;
    }

    if (race_status == NO_RACE_GOING)
    {
        xSemaphoreTake(dac_access_semaphore,0); /* lock DAC access */    
        dac_output_voltage(DAC_CHANNEL_1, 0);   /* stop car */
    }
    else if (race_status == RACE_GOING)
    {
        xSemaphoreGive(dac_access_semaphore);   /* free DAC */
    }

    if (light_state != '0')
    {
        send_data_wirelessly(race_status);  /* transmit race status, but only when state is not idle state, since that one does not give a good indication of where the state machine inside the CU is. */
    }
}