
# SOC of total voltage & current

float32 cumulative_total_voltage  # unit: V
float32 gather_total_voltage  # unit: V
float32 current  # unit: A
float32 soc  # unit: %

# Maximum & Minimum voltage

float32 maximum_cell_voltage_value  # unit: V
uint8 no_of_cell_with_maximum_voltage  # unit: number
float32 minimum_cell_voltage_value  # unit: V
uint8 no_of_cell_with_minimum_voltage  # unit: number

# Maximum & Minimum temperature

int8 maximum_temperature_value  # unit: °C
uint8 maximum_temperature_cell_no  # unit: number
int8 minimum_temperature_value  # unit: °C
uint8 minimum_temperature_cell_no  # unit: number

# Charge & discharge MOS status

uint8 state  # unit: number (0 = stationary  1 = charge  2 = discharge)
uint8 charge_mos_state
uint8 discharge_mos_status
uint8 bms_life  # unit: number
float32 remain_capacity  # unit: Ah

# Status information 1

uint8 no_of_battery_string  # unit: number
uint8 no_of_temperature  # unit: number
bool charger_status  # unit: boolean (false = disconnect  true = access)
bool load_status  # unit: boolean (false = disconnect  true = access)
bool[4] di_state  # unit: boolean (di_state[0 -> 3] = di_1_state -> di_4_state)
bool[4] do_state  # unit: boolean (do_state[0 -> 3] = do_1_state -> do_4_state)

# Cell voltage 1~48

float32[] cell_voltage  # unit: V

# Cell temperature 1~16

int8[] cell_temperature  # unit: °C

# Cell balance State 1~48

bool[48] cell_balance_state  # unit: boolean (false = closed  true = open)

# Battery failure status

bool cell_volt_high_level_1
bool cell_volt_high_level_2
bool cell_volt_low_level_1
bool cell_volt_low_level_2
bool sum_volt_high_level_1
bool sum_volt_high_level_2
bool sum_volt_low_level_1
bool sum_volt_low_level_2

bool chg_temp_high_level_1
bool chg_temp_high_level_2
bool chg_temp_low_level_1
bool chg_temp_low_level_2
bool dischg_temp_high_level_1
bool dischg_temp_high_level_2
bool dischg_temp_low_level_1
bool dischg_temp_low_level_2

bool chg_overcurrent_level_1
bool chg_overcurrent_level_2
bool dischg_overcurrent_level_1
bool dischg_overcurrent_level_2
bool soc_high_level_1
bool soc_high_level_2
bool soc_low_level_1
bool soc_low_level_2

bool diff_volt_level_1
bool diff_volt_level_2
bool diff_temp_level_1
bool diff_temp_level_2

bool chg_mos_temp_high_alarm
bool dischg_mos_temp_high_alarm
bool chg_mos_temp_sensor_err
bool dischg_mos_temp_sensor_err
bool chg_mos_adhesion_err
bool dischg_mos_adhesion_err
bool chg_mos_open_circuit_err
bool dischg_mos_open_circuit_err

bool afe_collect_chip_err
bool voltage_collect_dropped
bool cell_temp_sensor_err
bool eeprom_err
bool rtc_err
bool precharge_failure
bool communication_failure
bool internal_communication_failure

bool current_module_fault
bool sum_voltage_detect_fault
bool short_circuit_protect_fault
bool low_volt_forbidden_chg_fault

uint8 fault_code
