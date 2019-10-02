typedef enum {international=0, dwd, gnss, map_matching, comparing} State_type; //This enumeration contains all the main states
void internationaleFormel(), DWD(), gps(), MAP_MATCHING(), COMPARE();
void (*state_table[])() = {internationaleFormel, DWD, gps, MAP_MATCHING, COMPARE}; //This table conatins a pointer to the function to call in each state
State_type curr_state;
State_type prev_state;