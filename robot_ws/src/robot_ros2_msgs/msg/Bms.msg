
bool is_offline  # unit: boolean (false = online  true = offline)
bool is_warning  # unit: boolean (false = no  true = yes)

bool charge_mos_is_open  # unit: boolean (false = close  true = open)
bool discharge_mos_is_open  # unit: boolean (false = close  true = open)

uint8 state  # unit: number (0 = stationary  1 = charge  2 = discharge)
float32 voltage  # unit: V
float32 current  # unit: A
float32 soc  # unit: %
float32 remain_capacity  # unit: Ah

int8 maximum_temperature_value  # unit: °C
int8 minimum_temperature_value  # unit: °C

uint8[] warning
