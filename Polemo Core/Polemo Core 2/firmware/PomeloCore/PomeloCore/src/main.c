#include <asf.h>
#include <string.h>
#include <stdarg.h>
#include "conf_clocks.h"
#include "sio2host.h"
#include "nvm_params.h"

#define NVM_RWW				0x00400000

// Bootloader interface
#define DBL_TAP_PTR ((volatile uint32_t *)(HSRAM_ADDR + HSRAM_SIZE - 4))
#define DBL_TAP_MAGIC 0xf01669ef // Randomly selected, adjusted to have first and last bit set
#define SPECTRUM_LEN		1024
#define LIST_FIFO_LEN		128
#define UART_BUFFER_LEN		128

// bit numbers
#define LIST_UART_PULSE			0
#define LIST_UART_FAST_PULSE	1
#define LIST_UART_ENERGY		2
#define LIST_UART_ENERGY_TS		3
#define LIST_USB_PULSE			4
#define LIST_USB_ENERGY			5
#define LIST_USB_ENERGY_TS		6

typedef enum _UsbState_t
{
	STATE_CMD_IDLE,
	STATE_CMD_PARAM
}CmdState_t;


static struct core_params coreParams;
static struct physics_params physicsParams;

static float hvSipmVolts;

#define FILTER_LEN			4
extern const uint16_t pMove[1024][9];

static struct adc_module adc_instance_app;
static struct dac_module dac_instance_app;
static struct rtc_module rtc_instance_app;
struct spi_module spi_instance_app;
static struct i2c_master_module i2c_instance_app;
static struct tc_module tc_instance_app;
static struct tcc_module tcc_instance_app_pwm;

volatile static uint32_t hvloadTime;
volatile static uint32_t hvloadOn;
volatile static uint32_t hvloadOff;
static bool hvBoost;

volatile static uint32_t gammaTime;
volatile static uint32_t gammaCounts;
volatile static uint32_t gammaSum;
volatile static uint64_t gammaSumSquare;

volatile static uint32_t gammaPulseN, gammaPulseCoincidenceN;
volatile static uint32_t gammaPulseTstart, gammaPulseTstop;
volatile static bool gammaDaqRun, daq_enabled;
volatile static uint32_t gammaSpectrum[SPECTRUM_LEN];
volatile static uint32_t gammaSpectrumCoincidence[SPECTRUM_LEN];
volatile static uint16_t gammaFifo[LIST_FIFO_LEN];
volatile static uint32_t gammaFifoTs[LIST_FIFO_LEN];
volatile static uint16_t gammaFifoHead, gammaFifoTail;
volatile static bool gammaCoincidence;
volatile static bool gammaSyncTrig;
volatile static uint8_t gammaPulseChar;

volatile static uint8_t listOut;
volatile static bool temp_comp_run;

volatile bool usb_connected, can_sleep;
volatile bool isr_vbus;

static CmdState_t cmdState;

static struct usart_module app_uart_module;
static volatile uint8_t rxBufHead, rxBufTail;
static volatile uint8_t rxBuf[UART_BUFFER_LEN];

static void reset_to_bootloader(void);
void pomelo_printf(uint8_t iFace, const char* str);
int pomelo_sprintf(uint8_t iFace, const char * format, ... );


void USART_ISR_VECT(uint8_t instance)
{
	static uint16_t data;
	usart_read_wait(&app_uart_module, &data);

	if (rxBufHead != rxBufTail)
	{
		rxBuf[rxBufHead] = (uint8_t)data;
		rxBufHead = (rxBufHead + 1) % UART_BUFFER_LEN;
	}
}

bool uart_available()
{
	if ((rxBufTail + 1) % UART_BUFFER_LEN != rxBufHead) return true;
	else return false;
}

uint8_t uart_rx()
{
	rxBufTail = (rxBufTail + 1) % UART_BUFFER_LEN;
	return rxBuf[rxBufTail];
}

uint8_t uart_tx(uint8_t *data, uint8_t length)
{
	status_code_genare_t status;

	do {
		status = usart_serial_write_packet(&app_uart_module, (const uint8_t *)data, length);
	} while (status != STATUS_OK);
	return length;
}


void uart_init()
{
	struct usart_config host_uart_config;
	usart_get_config_defaults(&host_uart_config);
	host_uart_config.mux_setting = USART_RX_1_TX_0_XCK_1;
	host_uart_config.pinmux_pad0 = PINMUX_PA12C_SERCOM2_PAD0;
	host_uart_config.pinmux_pad1 = PINMUX_PA13C_SERCOM2_PAD1;
	host_uart_config.pinmux_pad2 = PINMUX_UNUSED;
	host_uart_config.pinmux_pad3 = PINMUX_UNUSED;
	//host_uart_config.baudrate    = 115200;
	host_uart_config.baudrate    = 921600;
	host_uart_config.run_in_standby = true;
	stdio_serial_init(&app_uart_module, SERCOM2, &host_uart_config);
	usart_enable(&app_uart_module);
	/* Enable transceivers */
	usart_enable_transceiver(&app_uart_module, USART_TRANSCEIVER_TX);
	usart_enable_transceiver(&app_uart_module, USART_TRANSCEIVER_RX);

	rxBufHead = 1;
	rxBufTail = 0;
	_sercom_set_handler(2, USART_ISR_VECT);
	SERCOM2->USART.INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SERCOM2);
}


static void daq_stop(void)
{
	gammaPulseTstop = rtc_count_get_count(&rtc_instance_app);
	gammaDaqRun = false;
}

static void isr_vbus_callback(void)
{
	volatile uint8_t a;
	a = port_pin_get_input_level(PIN_PA27);
	isr_vbus = true;
}

static inline void timer_updateValues_unsafe()
{
	uint32_t timerCounts;
	timerCounts = tc_get_count_value(&tc_instance_app);
	gammaTime += timerCounts;
	hvloadTime += timerCounts;
	tc_set_count_value(&tc_instance_app, 0);
}


static void synchronizer_callback(void)
{
	if (listOut != 0)
	{
		if (gammaFifoHead != gammaFifoTail)
		{
			timer_updateValues_unsafe();

			gammaFifo[gammaFifoHead] = 9999;
			gammaFifoTs[gammaFifoHead] = gammaTime;
			gammaFifoHead++;
			if (gammaFifoHead >= LIST_FIFO_LEN)
			{
				gammaFifoHead = 0;
			}
		}
	}
}


static void trigger_callback(void)
{
	static uint16_t adc_val, rnd, i;
	static volatile bool pinValue;
	pinValue = port_pin_get_input_level(PIN_PA18);

	if (daq_enabled)
	{
		adc_start_conversion(&adc_instance_app);
		while (adc_read(&adc_instance_app, &adc_val) == STATUS_BUSY) ;

		// Clear peak detector capacitor
		port_pin_set_output_level(PIN_PA08, 1);
		port_pin_set_output_level(PIN_PA15, 1);
		nop(); nop(); nop(); nop(); nop(); // 5us. Aaaactually almost nothing
		port_pin_set_output_level(PIN_PA15, 0);
		port_pin_set_output_level(PIN_PA08, 0);


		if (gammaDaqRun)
		{
			port_pin_set_output_level(PIN_PA19, 0);
			
			adc_val >>= 2;
			
			if (pMove[adc_val][FILTER_LEN] > 0)
			{
				// Bin is a donor, figure out if we give it away
				if ((rand() % 32768) <= pMove[adc_val][FILTER_LEN])
				{
					// We give sample away. Let's figure out where
					rnd = rand() % 32768;
						
					for (i = 0; i < 2*FILTER_LEN+1; i++)
					{
						if (rnd <= pMove[adc_val][i]) break;
					}
					adc_val = adc_val - FILTER_LEN + i;
				}
			}
			
			gammaSpectrum[adc_val]++;
			gammaPulseN++;

			if (gammaCoincidence)
			{
				if (port_pin_get_input_level(PIN_PA20))
				{
					gammaSpectrumCoincidence[adc_val]++;
					gammaPulseCoincidenceN++;
					if (listOut != 0)
					{
						if (gammaFifoHead != gammaFifoTail)
						{
							timer_updateValues_unsafe();

							gammaFifo[gammaFifoHead] = adc_val;
							gammaFifoTs[gammaFifoHead] = gammaTime;
							gammaFifoHead++;
							if (gammaFifoHead >= LIST_FIFO_LEN)
							{
								gammaFifoHead = 0;
							}
						}
					}
					
					// Loop while monitor line is still high
					while (port_pin_get_input_level(PIN_PA20))
					{
						// Assert our ACK line if downstream detectors are also in ACK
						port_pin_set_output_level(PIN_PB11, port_pin_get_input_level(PIN_PA09));
					}
					// Deassert our ACK line
					port_pin_set_output_level(PIN_PB11, 0);
				}
			}
			else
			{
				if (listOut != 0)
				{
					// By now we're committed to suboptimal timing in our ISR,
					// so maybe we go all in and send the UART click pulse
					// directly
					if ((listOut & (1<<LIST_UART_FAST_PULSE)) != 0)
					{
						uart_tx(&gammaPulseChar, 1);
					}
					else if (gammaFifoHead != gammaFifoTail)
					{
						timer_updateValues_unsafe();

						gammaFifo[gammaFifoHead] = adc_val;
						gammaFifoTs[gammaFifoHead] = gammaTime;
						gammaFifoHead++;
						if (gammaFifoHead >= LIST_FIFO_LEN)
						{
							gammaFifoHead = 0;
						}
					}
				}
			}
			
			gammaCounts++;
			gammaSum += adc_val;
			gammaSumSquare += adc_val*adc_val;
			
		}

		port_pin_set_output_level(PIN_PA19, 1);
		
		// Try to fix subthreshold structures. Retriggering of this interrupt can
		// cause a 2nd reading, after the peak detector had already been cleared
		extint_chan_clear_detected(2);
	}
	
}


static void rtc_callback(void)
{
	daq_stop();
}

static void rtc_callback_periodic(void)
{
	static uint8_t d = 0;
	
	d++;
	if (d == 5)
	{
		d = 0;
		temp_comp_run = true;
		gammaSyncTrig = true;
	}
}


static void timer_callback(struct tc_module *const module)
{
	gammaTime += 65536;
	hvloadTime += 65536;
}


static void hvload_callback(void)
{
	static uint32_t tOn = 0;
	static uint32_t tOff = 0;
	static volatile bool pinValue;
	pinValue = port_pin_get_input_level(PIN_PB10);
	timer_updateValues_unsafe();
	if (pinValue)
	{
		tOff = hvloadTime;
		hvloadTime = 0;
		if (tOn != 0)
		{
			hvloadOn = tOn;
			hvloadOff = tOff;
			tOn = 0;
			tOff = 0;
		}
	}
	else
	{
		tOn = hvloadTime;
		hvloadTime = 0;
	}
}


static float hvload_uA()
{
	float iOut;
	
	if ((hvloadTime < 65535) && (hvloadOff != 0))
	{
		iOut = coreParams.iMeas[1] / hvloadOff + coreParams.iMeas[0];
		// 5.6 MOhm resistor in LT3014 feedback loop steals some current too. But resistor value is loaded from parameter
		if (coreParams.iMeas[2] != 0) iOut -= (hvSipmVolts - 1.22)/coreParams.iMeas[2];
		if (iOut < 0) iOut = 0;
	}
	else
	{
		iOut = 9999;
	}
	
	return iOut;
}

static void timer_init_gamma(void)
{
	struct tc_config tc_conf;
	
	tc_get_config_defaults(&tc_conf);
	tc_conf.clock_source = GCLK_GENERATOR_4;
	tc_conf.counter_size = TC_COUNTER_SIZE_16BIT;
	tc_conf.clock_prescaler = TC_CLOCK_PRESCALER_DIV1;
	tc_conf.count_direction = TC_COUNT_DIRECTION_UP;
	tc_conf.oneshot = false;
	tc_conf.run_in_standby = true;
	tc_init(&tc_instance_app, TC0, &tc_conf);
	
	tc_register_callback(&tc_instance_app, timer_callback, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance_app, TC_CALLBACK_OVERFLOW);

	tc_enable(&tc_instance_app);
}

static void coincidences_reset(void)
{
	struct port_config pin_conf;

	ccl_module_reset();
	
	// All pins set as inputs with pull-down enabled
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_DOWN;

	// CCL2 and CCL0 inputs and outputs
	port_pin_set_config(PIN_PA23, &pin_conf);
	port_pin_set_config(PIN_PA22, &pin_conf);
	port_pin_set_config(PIN_PB22, &pin_conf);
	port_pin_set_config(PIN_PB09, &pin_conf);
	port_pin_set_config(PIN_PB23, &pin_conf);
	
	// Software clear mechanism inputs and output
	port_pin_set_config(PIN_PA20, &pin_conf);
	port_pin_set_config(PIN_PA09, &pin_conf);
	port_pin_set_config(PIN_PB11, &pin_conf);
	
	// Also handle synchronization config
	port_pin_set_config(PIN_PB02, &pin_conf);
}

static void coincidences_enable(void)
{
	struct ccl_config ccl_conf;
	struct ccl_lut_config ccl_lut_conf;
	struct system_pinmux_config pinmux_config;
	struct port_config pin_conf;

	// Coincidences synchronizer output
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PB02, &pin_conf);
	port_pin_set_output_level(PIN_PB02, 0);
	
	// ACK output
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PB11, &pin_conf);
	port_pin_set_output_level(PIN_PB11, 0);
	
	// Set PA22 as remote trigger input - CCL2 In0
	system_pinmux_get_config_defaults(&pinmux_config);
	pinmux_config.mux_position = 8;
	pinmux_config.direction    = SYSTEM_PINMUX_PIN_DIR_INPUT;
	pinmux_config.input_pull   = SYSTEM_PINMUX_PIN_PULL_DOWN;
	system_pinmux_pin_set_config(PIN_PA22, &pinmux_config);

	// Set PA23 as local trigger input - CCL2 In1
	system_pinmux_get_config_defaults(&pinmux_config);
	pinmux_config.mux_position = 8;
	pinmux_config.direction    = SYSTEM_PINMUX_PIN_DIR_INPUT;
	pinmux_config.input_pull   = SYSTEM_PINMUX_PIN_PULL_DOWN;
	system_pinmux_pin_set_config(PIN_PA23, &pinmux_config);

	// Set PB09 as coincidences output - CCL2 out
	system_pinmux_get_config_defaults(&pinmux_config);
	pinmux_config.mux_position = 8;
	pinmux_config.direction    = SYSTEM_PINMUX_PIN_DIR_OUTPUT_WITH_READBACK;
	system_pinmux_pin_set_config(PIN_PB09, &pinmux_config);

	// Set PB22 as ACK0 input - CCL0 In0
	system_pinmux_get_config_defaults(&pinmux_config);
	pinmux_config.mux_position = 8;
	pinmux_config.direction    = SYSTEM_PINMUX_PIN_DIR_INPUT;
	pinmux_config.input_pull   = SYSTEM_PINMUX_PIN_PULL_DOWN;
	system_pinmux_pin_set_config(PIN_PB22, &pinmux_config);

	// Set PB23 as coincidences output - CCL0 out
	system_pinmux_get_config_defaults(&pinmux_config);
	pinmux_config.mux_position = 8;
	pinmux_config.direction    = SYSTEM_PINMUX_PIN_DIR_OUTPUT_WITH_READBACK;
	system_pinmux_pin_set_config(PIN_PB23, &pinmux_config);

	ccl_module_reset();

	// Make coincidence trigger logic
	ccl_get_config_defaults(&ccl_conf);
	ccl_conf.run_in_standby = true;
	ccl_conf.clock_source = GCLK_GENERATOR_0;
	ccl_init(&ccl_conf);

	// LUT2: In0 Remote trigger, In1 Local trigger, In2 Lnk
	ccl_lut_get_config_defaults(&ccl_lut_conf);
	ccl_lut_conf.edge_selection_enable = false;
	ccl_lut_conf.event_input_enable = false;
	ccl_lut_conf.event_output_enable = false;
	ccl_lut_conf.filter_sel = CCL_LUT_FILTER_DISABLE;
	ccl_lut_conf.input0_src_sel = CCL_LUT_INPUT_SRC_IO;
	ccl_lut_conf.input1_src_sel = CCL_LUT_INPUT_SRC_IO;
	ccl_lut_conf.input2_src_sel = CCL_LUT_INPUT_SRC_LINK;
	ccl_lut_conf.truth_table_value = 0xF8;		// (In0 AND In1) OR In2
	ccl_lut_set_config(CCL_LUT_2, &ccl_lut_conf);

	// LUT0: In0 ACK, In1: Lnk; ACK0 reset, output to pin
	ccl_lut_get_config_defaults(&ccl_lut_conf);
	ccl_lut_conf.edge_selection_enable = false;
	ccl_lut_conf.event_input_enable = false;
	ccl_lut_conf.event_output_enable = false;
	ccl_lut_conf.filter_sel = CCL_LUT_FILTER_DISABLE;
	ccl_lut_conf.input0_src_sel = CCL_LUT_INPUT_SRC_IO;
	ccl_lut_conf.input1_src_sel = CCL_LUT_INPUT_SRC_LINK;
	ccl_lut_conf.input2_src_sel = CCL_LUT_INPUT_SRC_MASK;
	ccl_lut_conf.truth_table_value = 0x44;	// In1 AND NOT In0
	ccl_lut_set_config(CCL_LUT_0, &ccl_lut_conf);

	// LUT1: Pass
	ccl_lut_get_config_defaults(&ccl_lut_conf);
	ccl_lut_conf.edge_selection_enable = false;
	ccl_lut_conf.event_input_enable = false;
	ccl_lut_conf.event_output_enable = false;
	ccl_lut_conf.filter_sel = CCL_LUT_FILTER_DISABLE;
	ccl_lut_conf.input0_src_sel = CCL_LUT_INPUT_SRC_LINK;
	ccl_lut_conf.input1_src_sel = CCL_LUT_INPUT_SRC_MASK;
	ccl_lut_conf.input2_src_sel = CCL_LUT_INPUT_SRC_MASK;
	ccl_lut_conf.truth_table_value = 0x02;
	ccl_lut_set_config(CCL_LUT_1, &ccl_lut_conf);

	// LUT3: Pass
	ccl_lut_get_config_defaults(&ccl_lut_conf);
	ccl_lut_conf.edge_selection_enable = false;
	ccl_lut_conf.event_input_enable = false;
	ccl_lut_conf.event_output_enable = false;
	ccl_lut_conf.filter_sel = CCL_LUT_FILTER_DISABLE;
	ccl_lut_conf.input0_src_sel = CCL_LUT_INPUT_SRC_LINK;
	ccl_lut_conf.input1_src_sel = CCL_LUT_INPUT_SRC_MASK;
	ccl_lut_conf.input2_src_sel = CCL_LUT_INPUT_SRC_MASK;
	ccl_lut_conf.truth_table_value = 0x02;
	ccl_lut_set_config(CCL_LUT_3, &ccl_lut_conf);
	
	ccl_lut_enable(CCL_LUT_0);
	ccl_lut_enable(CCL_LUT_1);
	ccl_lut_enable(CCL_LUT_2);
	ccl_lut_enable(CCL_LUT_3);
	
	ccl_module_enable();
}


void hv_enable(void)
{
	struct system_pinmux_config pinmux_config;
	struct system_gclk_gen_config gclk_conf;
	struct tcc_config tcc_conf;

	if (hvBoost)
	{
		// Can do max ~300uA
		// To do: tune period and inductor duty cycle

		// Configure clock generator for TCC
		system_gclk_gen_get_config_defaults(&gclk_conf);
		gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC16M;
		gclk_conf.division_factor = 1;
		gclk_conf.output_enable = false;
		gclk_conf.run_in_standby = true;
		system_gclk_gen_set_config(GCLK_GENERATOR_5, &gclk_conf);
		system_gclk_gen_enable(GCLK_GENERATOR_5);
		
		// Configure and start PWM on PA10 with TCC
		tcc_get_config_defaults(&tcc_conf, TCC0);
		tcc_conf.counter.clock_source = GCLK_GENERATOR_5;
		tcc_conf.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
		tcc_conf.counter.period = 700;
		tcc_conf.compare.channel_function[2] = TCC_CHANNEL_FUNCTION_COMPARE;
		tcc_conf.compare.match[2] = 64;
		tcc_conf.compare.wave_polarity[2] = TCC_WAVE_POLARITY_1;
		tcc_conf.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
		tcc_conf.compare.wave_ramp = TCC_RAMP_RAMP1;
		tcc_conf.pins.wave_out_pin[2] = PIN_PA10F_TCC0_WO2;
		tcc_conf.pins.wave_out_pin_mux[2] = MUX_PA10F_TCC0_WO2;
		tcc_conf.pins.enable_wave_out_pin[2] = true;
		tcc_conf.run_in_standby = true;
	
		tcc_init(&tcc_instance_app_pwm, TCC0, &tcc_conf);
		tcc_enable(&tcc_instance_app_pwm);
	}
	else
	{
		// Connect clock generator output to pin PA10
		system_pinmux_get_config_defaults(&pinmux_config);
		pinmux_config.mux_position = 7;
		pinmux_config.direction    = PORT_PIN_DIR_OUTPUT;
		system_pinmux_pin_set_config(PIN_PA10, &pinmux_config);
	}
}


void hv_changeBoost(bool newHvBoost)
{
	// ONLY WHEN HV IS ALREADY RUNNING!
	
	struct system_pinmux_config pinmux_config;
	struct system_gclk_gen_config gclk_conf;
	struct tcc_config tcc_conf;

	if (newHvBoost == hvBoost) return;

	if (newHvBoost)
	{
		// Enable boost

		// Configure clock generator for TCC
		system_gclk_gen_get_config_defaults(&gclk_conf);
		gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC16M;
		gclk_conf.division_factor = 1;
		gclk_conf.output_enable = false;
		gclk_conf.run_in_standby = true;
		system_gclk_gen_set_config(GCLK_GENERATOR_5, &gclk_conf);
		system_gclk_gen_enable(GCLK_GENERATOR_5);
		
		// Configure and start PWM on PA10 with TCC
		tcc_get_config_defaults(&tcc_conf, TCC0);
		tcc_conf.counter.clock_source = GCLK_GENERATOR_5;
		tcc_conf.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
		tcc_conf.counter.period = 700;
		tcc_conf.compare.channel_function[2] = TCC_CHANNEL_FUNCTION_COMPARE;
		tcc_conf.compare.match[2] = 64;
		tcc_conf.compare.wave_polarity[2] = TCC_WAVE_POLARITY_1;
		tcc_conf.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
		tcc_conf.compare.wave_ramp = TCC_RAMP_RAMP1;

		tcc_conf.pins.wave_out_pin[2] = PIN_PA10F_TCC0_WO2;
		tcc_conf.pins.wave_out_pin_mux[2] = MUX_PA10F_TCC0_WO2;
		tcc_conf.pins.enable_wave_out_pin[2] = true;
		tcc_conf.run_in_standby = true;
		
		tcc_init(&tcc_instance_app_pwm, TCC0, &tcc_conf);
		tcc_enable(&tcc_instance_app_pwm);
	}
	else
	{
		// Connect clock generator output to pin PA10
		system_pinmux_get_config_defaults(&pinmux_config);
		pinmux_config.mux_position = 7;
		pinmux_config.direction    = PORT_PIN_DIR_OUTPUT;
		system_pinmux_pin_set_config(PIN_PA10, &pinmux_config);

		// Now switch off unnecessary parts
		tcc_disable(&tcc_instance_app_pwm);
		tcc_reset(&tcc_instance_app_pwm);
		system_gclk_gen_disable(GCLK_GENERATOR_5);
	}
	
	hvBoost = newHvBoost;
}

void hv_disable(void)
{
	struct port_config pin_conf;
	struct system_gclk_gen_config gclk_conf;

	if (hvBoost)
	{
		tcc_disable(&tcc_instance_app_pwm);
		tcc_reset(&tcc_instance_app_pwm);
		system_gclk_gen_disable(GCLK_GENERATOR_5);
	}
	
	port_get_config_defaults(&pin_conf);

	// HV PWM output 0
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA10, &pin_conf);
	port_pin_set_output_level(PIN_PA10, 0);
	
	// Pulse HV crowbar
	port_pin_set_output_level(PIN_PA21, 0);
	delay_ms(10);
	port_pin_set_output_level(PIN_PA21, 1);
}


static void adc_init_gamma(void)
{
	struct adc_config adc_conf;
	struct system_voltage_references_config conf_intref;	
	
	adc_get_config_defaults(&adc_conf);
	
	adc_conf.clock_source = GCLK_GENERATOR_0;
	adc_conf.clock_prescaler = ADC_CLOCK_PRESCALER_DIV2;
	adc_conf.reference = ADC_REFCTRL_REFSEL_INTREF;
	adc_conf.positive_input = ADC_POSITIVE_INPUT_PIN7;
	adc_conf.negative_input = ADC_NEGATIVE_INPUT_GND;
	adc_conf.sample_length = 4;
	adc_conf.resolution = ADC_RESOLUTION_12BIT;
	adc_conf.freerunning = false;
	adc_conf.event_action = ADC_EVENT_ACTION_DISABLED;
	adc_conf.run_in_standby = false;
	
	adc_init(&adc_instance_app, ADC, &adc_conf);

	system_voltage_reference_get_config_defaults(&conf_intref);
	conf_intref.sel = SYSTEM_VOLTAGE_REFERENCE_2V0;
	conf_intref.run_in_standby = true;
	system_voltage_reference_set_config(&conf_intref);
	system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_OUTPUT);
	system_voltage_reference_disable(SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE);
	
	adc_enable(&adc_instance_app);
}


static void dac_init_gamma_hv(void)
{
	struct dac_config dac_conf;
	struct dac_chan_config dac_chan_conf;
	
	dac_get_config_defaults(&dac_conf);
	dac_conf.differential_mode = false;
	dac_conf.reference = DAC_REFERENCE_INTREF;
	dac_conf.clock_source = GCLK_GENERATOR_2;
	dac_init(&dac_instance_app, DAC, &dac_conf);
	
	dac_chan_get_config_defaults(&dac_chan_conf);
	
	dac_chan_conf.left_adjust = false;
	dac_chan_conf.run_in_standby = true;
	dac_chan_conf.dither_mode = false;
	dac_chan_conf.refresh_period = 2;
	dac_chan_conf.current = DAC_CURRENT_100K;
	
	dac_chan_set_config(&dac_instance_app, DAC_CHANNEL_0, &dac_chan_conf);
	dac_chan_enable(&dac_instance_app, DAC_CHANNEL_0);
	
	dac_chan_set_config(&dac_instance_app, DAC_CHANNEL_1, &dac_chan_conf);
	dac_chan_enable(&dac_instance_app, DAC_CHANNEL_1);
	
	dac_enable(&dac_instance_app);
}

static void i2c_init(void)
{
	enum status_code status;
	struct i2c_master_config config_i2c_master;

	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.buffer_timeout = 100;
	config_i2c_master.pinmux_pad0    = PINMUX_PA16C_SERCOM1_PAD0;
	config_i2c_master.pinmux_pad1    = PINMUX_PA17C_SERCOM1_PAD1;
	config_i2c_master.run_in_standby = 0;
	status = i2c_master_init(&i2c_instance_app, SERCOM1, &config_i2c_master);
	i2c_master_enable(&i2c_instance_app);
}

static void read_physics(uint8_t n, uint8_t *data)
{
	enum status_code status;
	uint8_t timeOut;
	uint8_t addr;
	struct i2c_master_packet master_packet = {
		.address     = 0x50,
		.data_length = 1,
		.data        = &addr,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	addr = 0;

	timeOut = 5;
	while (timeOut > 0)
	{
		status = i2c_master_write_packet_wait_no_stop(&i2c_instance_app, &master_packet);
		if (status == STATUS_OK) break;
		delay_ms(10);
		timeOut--;
	}
	if (timeOut == 0) return;
	
	master_packet.data_length = n;
	master_packet.data = data;
	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);
}

static void write_physics(uint8_t page, uint8_t n, uint8_t *data)
{
	enum status_code status;
	uint8_t addr;
	uint8_t timeOut, i;
	struct i2c_master_packet master_packet = {
		.address     = 0x50,
		.data_length = 1,
		.data        = &addr,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	if ((addr & 0x0F) != 0) return;
	if (n > 16) return;

	addr = page << 4;

	timeOut = 5;
	while (timeOut > 0)
	{
		status = i2c_master_write_packet_wait_no_stop(&i2c_instance_app, &master_packet);
		if (status == STATUS_OK) break;
		delay_ms(10);
		timeOut--;
	}
	if (timeOut == 0) return;

	for (i = 0; i < n; i++) status = i2c_master_write_byte(&i2c_instance_app, data[i]);
	i2c_master_send_stop(&i2c_instance_app);
}


static void init_physics()
{
	uint16_t writeBytes, i;
	struct physics_params params = {
		.sipm_vMin = 0,
		.sipm_vMax = 45,
		.sipm_v0deg = 38.0,
		.sipm_vTempComp = 0.03593,
		.ecal = {0, 1, 0},
		.uSvph_constant = 2.09534976,
		//.detString = "CapeScint LaBr-14x25c-SiPM-T",
		//.detString = "3\" Adapter, Broadcom 8x8mm2 SiPM",
		.detString = "CsI(Tl) 2cm3, Broadcom 4x4mm2 SiPM",
		.tempType = 1,
		.version = 0x01,
	};
	
	writeBytes = sizeof(params);

	// Write structure to I2C eeprom
	for (i = 0; i < writeBytes/16; i++)
	{
		write_physics(i, 16, (uint8_t*)(&params) + i*16);
	}

	if (writeBytes > i*16)
	{
		// We need to write a bit more, but not a full page
		write_physics(i, writeBytes - i*16, (uint8_t*)(&params) + i*16);
	}

}


static float read_temp_pct2075(void)
{
	enum status_code status;
	uint8_t i2c_buffer[2];
	int16_t output;
	struct i2c_master_packet master_packet = {
		.address     = 0x49,
		.data_length = 2,
		.data        = i2c_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	i2c_buffer[0] = 0; // read from temperaure register @ addr 0
	master_packet.data_length = 1;

	status = i2c_master_write_packet_wait_no_stop(&i2c_instance_app, &master_packet);

	master_packet.data_length = 2;

	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);
	output = (int16_t)(((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1]);

	output = output >> 5;

	return output*0.125;
}


static void temp_init_tmp116(void)
{
	enum status_code status;
	uint8_t i2c_buffer[3];
	struct i2c_master_packet master_packet = {
		.address     = 0x48,
		.data_length = 2,
		.data        = &i2c_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	master_packet.data_length = 1;
	i2c_buffer[0] = 0; // temperature register
	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);

	master_packet.data_length = 2;

	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);

}


static float read_temp_tmp116(void)
{
	enum status_code status;
	uint8_t i2c_buffer[3];
	int16_t output;
	struct i2c_master_packet master_packet = {
		.address     = 0x48,
		.data_length = 2,
		.data        = i2c_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);
	output = (int16_t)((i2c_buffer[0] << 8) | i2c_buffer[1]);

	return output*0.0078125;
}


static void temp_init_tmp451(void)
{
	enum status_code status;
	uint8_t i2c_buffer[3];
	struct i2c_master_packet master_packet = {
		.address     = 0x4C,
		.data_length = 3,
		.data        = &i2c_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	// set range. config.bit2 = 1
	master_packet.data_length = 2;
	i2c_buffer[0] = 9; // config write
	i2c_buffer[1] = 0x84; // MASK1 = 1, continuous conversions, extended range
	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);
}


static float read_temp_tmp451(void)
{
	enum status_code status;
	uint8_t i2c_buffer[3];
	int16_t output;
	
	struct i2c_master_packet master_packet = {
		.address     = 0x4C,
		.data_length = 1,
		.data        = i2c_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	master_packet.data_length = 1;

	i2c_buffer[0] = 0; // temp high
	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);
	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);
	output = i2c_buffer[0];

	i2c_buffer[0] = 0x15; // temp low
	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);
	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);

	output <<= 4;
	output |= i2c_buffer[0] >> 4;
	
	output -= 64*16;

	return ((int16_t)output)*0.0625;
}

static void temp_init(void)
{
	switch (physicsParams.tempType)
	{
		case 0:
			break;
		case 1:
			temp_init_tmp116();
			break;
		case 2:
			temp_init_tmp451();
			break;
	}
}

static float read_temp(void)
{
	switch (physicsParams.tempType)
	{
		case 0:
			return read_temp_pct2075();
			break;
		case 1:
			return read_temp_tmp116();
			break;
		case 2:
			return read_temp_tmp451();
			break;
		default:
			return 0;
			break;
	}
	return 0;
}

static void update_hv_temp(bool force)
{
	static uint16_t oldDacHvValue = 0;
	float temp, dacHvValue;
	temp = read_temp();
	// Maybe need to update with values from datasheet numbers: 0.03633 * dDAC/dV. Comes out to -10.0961 for one board
	//delta = (int16_t)(-5.8784*(temp - tempCompTemp));		// Calculated from Am-241 data
	hvSipmVolts = physicsParams.sipm_v0deg + physicsParams.sipm_vTempComp*temp;
	if (hvSipmVolts > physicsParams.sipm_vMax) hvSipmVolts = physicsParams.sipm_vMax;
	if (hvSipmVolts < physicsParams.sipm_vMin) hvSipmVolts = physicsParams.sipm_vMin;

	dacHvValue = coreParams.vDac[0] + coreParams.vDac[1]*hvSipmVolts;
	if (dacHvValue > 4095) dacHvValue = 4095;
	if (dacHvValue < 0) dacHvValue = 0;
	if (force || ((uint16_t)dacHvValue != oldDacHvValue))
	{
		oldDacHvValue = (uint16_t)dacHvValue;
		dac_chan_write(&dac_instance_app, DAC_CHANNEL_0, oldDacHvValue);
	}
}

static void eic_init()
{
	struct extint_chan_conf eint_chan_conf;

	// Mihai: I messed up EIC_Handler() prioritize EXTINT2 BIGTIME

	// USB
	extint_chan_get_config_defaults(&eint_chan_conf);
	eint_chan_conf.gpio_pin           = PIN_PA27A_EIC_EXTINT15;
	eint_chan_conf.gpio_pin_mux       = MUX_PA27A_EIC_EXTINT15;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_RISING;
	eint_chan_conf.gpio_pin_pull      = SYSTEM_PINMUX_PIN_PULL_NONE;
	eint_chan_conf.filter_input_signal = false;
	eint_chan_conf.enable_async_edge_detection = true;
	extint_chan_set_config(15, &eint_chan_conf);

	extint_register_callback(isr_vbus_callback, 15, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(15, EXTINT_CALLBACK_TYPE_DETECT);

	// Trigger
	extint_chan_get_config_defaults(&eint_chan_conf);
	eint_chan_conf.gpio_pin           = PIN_PA18A_EIC_EXTINT2;
	eint_chan_conf.gpio_pin_mux       = MUX_PA18A_EIC_EXTINT2;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_FALLING;
	eint_chan_conf.gpio_pin_pull      = SYSTEM_PINMUX_PIN_PULL_NONE;
	eint_chan_conf.filter_input_signal = false;
	eint_chan_conf.enable_async_edge_detection = true;
	extint_chan_set_config(2, &eint_chan_conf);

	extint_register_callback(trigger_callback, 2, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(2, EXTINT_CALLBACK_TYPE_DETECT);

	// Coincidences synchronizer
	extint_chan_get_config_defaults(&eint_chan_conf);
	eint_chan_conf.gpio_pin           = PIN_PB08A_EIC_EXTINT8;
	eint_chan_conf.gpio_pin_mux       = MUX_PB08A_EIC_EXTINT8;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_RISING;
	eint_chan_conf.gpio_pin_pull      = SYSTEM_PINMUX_PIN_PULL_DOWN;
	eint_chan_conf.filter_input_signal = false;
	eint_chan_conf.enable_async_edge_detection = true;
	extint_chan_set_config(8, &eint_chan_conf);

	extint_register_callback(synchronizer_callback, 8, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);


	// HV load measurement using PB10
	extint_chan_get_config_defaults(&eint_chan_conf);
	eint_chan_conf.gpio_pin           = PIN_PB10A_EIC_EXTINT10;
	eint_chan_conf.gpio_pin_mux       = MUX_PB10A_EIC_EXTINT10;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_BOTH;
	eint_chan_conf.gpio_pin_pull      = SYSTEM_PINMUX_PIN_PULL_NONE;
	eint_chan_conf.filter_input_signal = false;
	eint_chan_conf.enable_async_edge_detection = true;
	extint_chan_set_config(10, &eint_chan_conf);

	extint_register_callback(hvload_callback, 10, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(10, EXTINT_CALLBACK_TYPE_DETECT);

}

static void rtc_init_gamma(void)
{
	struct rtc_count_config rtc_conf;

	rtc_count_get_config_defaults(&rtc_conf);
	rtc_conf.prescaler = RTC_COUNT_PRESCALER_DIV_1024;
	rtc_conf.mode = RTC_COUNT_MODE_32BIT;
	rtc_conf.clear_on_match = false;
	rtc_conf.enable_read_sync = true;
	rtc_count_init(&rtc_instance_app, RTC, &rtc_conf);
	
	rtc_count_reset(&rtc_instance_app);
	rtc_count_init(&rtc_instance_app, RTC, &rtc_conf);
	
	rtc_count_set_count(&rtc_instance_app, 0);
	rtc_count_enable(&rtc_instance_app);
	rtc_count_set_compare(&rtc_instance_app, 0xFFFFFFFF, 0);
	
	rtc_count_register_callback(&rtc_instance_app, rtc_callback, RTC_COUNT_CALLBACK_COMPARE_0);
	rtc_count_enable_callback(&rtc_instance_app, RTC_COUNT_CALLBACK_COMPARE_0);

	rtc_count_register_callback(&rtc_instance_app, rtc_callback_periodic, RTC_COUNT_CALLBACK_PERIODIC_INTERVAL_7);
	rtc_count_enable_callback(&rtc_instance_app, RTC_COUNT_CALLBACK_PERIODIC_INTERVAL_7);
	
}

static void command_response(uint8_t iFace, int8_t response)
{
	pomelo_sprintf(iFace, "{\"type\":\"response\", \"payload\":%d}\n", response);
}


static int8_t adc_write_constants(uint16_t nRow, uint8_t *data)
{
	uint32_t baseAddress = (uint32_t)(pMove);
	uint16_t i;
	enum status_code sc;

	sc = nvm_erase_row(baseAddress + nRow * NVMCTRL_PAGE_SIZE * NVMCTRL_ROW_PAGES);
	if (sc != STATUS_OK) return -1;

	for (i = 0; i < NVMCTRL_ROW_PAGES; i++)
	{
		sc = nvm_write_buffer(baseAddress + nRow * NVMCTRL_PAGE_SIZE * NVMCTRL_ROW_PAGES + i * NVMCTRL_PAGE_SIZE, &data[i * NVMCTRL_PAGE_SIZE], NVMCTRL_PAGE_SIZE);
		if (sc != STATUS_OK) return -2;
	}
	return 0;
}

static int8_t adc_compute_constants(uint32_t *calibSpectrum)
{
	uint16_t rows[256];		// 256 = NVMCTRL_PAGE_SIZE (64) * NVMCTRL_ROW_PAGES (4) * number_of_rows_in_my_buffer (2) / bytes_per_uint16_t (2)
	uint16_t pMoveBin[2*FILTER_LEN+1];
	int32_t totalToMove;
	uint16_t i, j, p, nRow;
	double avgVal;
	uint8_t rowsPtr;
	int8_t retVal;
	
	retVal = 0;
	
	nRow = 0;
	rowsPtr = 0;
	
	for (i = 0; i < 1024; i++)
	{
		// Default values that will stay like this for attractor bins, or for first and last bins
		for (j = 0; j < 2*FILTER_LEN+1; j++) pMoveBin[j] = 0;

		if ((i >= FILTER_LEN + 2) && (i <= 1023 - FILTER_LEN - 2))
		{
			p = i - FILTER_LEN;
		
			// Value that ideally all bins should contain
			avgVal = 0;
			for (j = p; j < p + 2*FILTER_LEN+1; j++)
			{
				avgVal += calibSpectrum[j];
			}
			avgVal /= 2*FILTER_LEN+1;

		
			if ((double)calibSpectrum[i] - avgVal > 0)
			{
				// Donor bin, pMove is probability to give current sample away to an acceptor
				pMoveBin[FILTER_LEN] = ((double)calibSpectrum[i] - avgVal)*32768.0/calibSpectrum[i];

				// Count number of samples above average (that will be transferred from
				// donors to acceptors) in the current filter context
				totalToMove = 0;
				for (j = 0; j < 2*FILTER_LEN+1; j++)
				{
					if ((double)calibSpectrum[p + j] - avgVal > 0) totalToMove += (double)calibSpectrum[p + j] - avgVal;
				}

				// pMove is probability that that neighbour receives a sample in the current context
				for (j = 0; j < 2*FILTER_LEN+1; j++)
				{
					if (j == FILTER_LEN) continue;
				
                    if (j == FILTER_LEN + 1) pMoveBin[j] = pMoveBin[j-2];
                    else if (j != 0) pMoveBin[j] = pMoveBin[j-1];
					if (avgVal - calibSpectrum[p + j] > 0)
					{
						// Acceptor bin. Populate pMove as sum
						pMoveBin[j] += (avgVal - (double)calibSpectrum[p + j])*32768.0/totalToMove;
					}
				}
			}
		}

		// copy pMoveBin data into rows[] array.
		for (j = 0; j < 2*FILTER_LEN+1; j++)
		{
			rows[rowsPtr] = pMoveBin[j];
			rowsPtr++;
		}
		
		// If we have enough data to fill a NVM row, do that now.
		if ( (rowsPtr >= 128) && (rowsPtr - (2*FILTER_LEN+1) < 128) )
		{
			// We filled in the first half of the rows buffer, write it to NVM
			retVal = adc_write_constants(nRow, (uint8_t*)&rows[0]);
			if (retVal != 0) return retVal;
			nRow++;
		}

		if (rowsPtr < 2*FILTER_LEN+1 )
		{
			// Filled in 2nd half, write that to NVM
			retVal = adc_write_constants(nRow, (uint8_t*)&rows[128]);
			if (retVal != 0) return retVal;
			nRow++;
		}
	}
	return retVal;
}


static void adc_calib(uint8_t iFace)
{
	uint16_t adc_val, i, j;
	struct port_config pin_conf;
	int8_t retVal;
	
	retVal = 0;
	
	port_get_config_defaults(&pin_conf);
	
	adc_set_positive_input(&adc_instance_app, ADC_POSITIVE_INPUT_PIN4);
	
	for (i = 0; i < SPECTRUM_LEN; i++) gammaSpectrum[i] = 0;
	
	// Ramp generator enable
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA03, &pin_conf);
	port_pin_set_output_level(PIN_PA03, 1);

	// Ramp generator trigger
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA11, &pin_conf);
	port_pin_set_output_level(PIN_PA11, 1);

	adc_val = 4095;
	while (adc_val > 100)
	{
		adc_start_conversion(&adc_instance_app);
		while (adc_read(&adc_instance_app, &adc_val) == STATUS_BUSY) ;
	}
	
	for (i = 0; i < 1000; i++)
	{
		port_pin_set_output_level(PIN_PA11, 0);
		while (adc_val < 4000)
		{
			adc_start_conversion(&adc_instance_app);
			while (adc_read(&adc_instance_app, &adc_val) == STATUS_BUSY) ;
			gammaSpectrum[adc_val >> 2]++;
		}

		for (j = 0; j < 2500; j++)
		{
			adc_start_conversion(&adc_instance_app);
			while (adc_read(&adc_instance_app, &adc_val) == STATUS_BUSY) ;
			gammaSpectrum[adc_val >> 2]++;
		}

		port_pin_set_output_level(PIN_PA11, 1);

		while (adc_val > 100)
		{
			adc_start_conversion(&adc_instance_app);
			while (adc_read(&adc_instance_app, &adc_val) == STATUS_BUSY) ;
			gammaSpectrum[adc_val >> 2]++;
		}

		for (j = 0; j < 2500; j++)
		{
			adc_start_conversion(&adc_instance_app);
			while (adc_read(&adc_instance_app, &adc_val) == STATUS_BUSY) ;
			gammaSpectrum[adc_val >> 2]++;
		}
		
		if (i % 10 == 0) pomelo_sprintf(iFace, "{\"type\":\"calib_progress\", \"payload\":%d}\n", i/10);
	}

	port_pin_set_output_level(PIN_PA11, 0);
	port_pin_set_output_level(PIN_PA03, 0);

	adc_set_positive_input(&adc_instance_app, ADC_POSITIVE_INPUT_PIN7);

	// Ramp generator input
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_PA04, &pin_conf);

	pomelo_printf(iFace, "{\"type\":\"calib_spectrum\", \"payload\":{\"data\":[");
	for (i = 0; i < SPECTRUM_LEN - 1; i++)
	{
		pomelo_sprintf(iFace, "%lu,", (unsigned long)gammaSpectrum[i]);
	}
	pomelo_sprintf(iFace, "%lu]}}\n", (unsigned long)gammaSpectrum[SPECTRUM_LEN - 1]);

	retVal = adc_compute_constants(gammaSpectrum);
	
	command_response(iFace, retVal);
}


void unused_pins_init(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	// All pins output low
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;

	port_pin_set_config(PIN_PA30, &pin_conf);
	port_pin_set_output_level(PIN_PA30, 1);

	port_pin_set_config(PIN_PA31, &pin_conf);
	port_pin_set_output_level(PIN_PA31, 1);

	// USB pins input
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_DOWN;
	port_pin_set_config(PIN_PA24, &pin_conf);
	port_pin_set_config(PIN_PA25, &pin_conf);
}


static void main_clock_select_osc16m(void)
{
	struct system_gclk_gen_config gclk_conf;

	/* Select OSC16M as mainclock */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC16M;
	gclk_conf.division_factor = 1;
	gclk_conf.run_in_standby = true;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);
	if (CONF_CLOCK_OSC16M_ON_DEMAND) {
		OSCCTRL->OSC16MCTRL.reg |= OSCCTRL_OSC16MCTRL_ONDEMAND;
	}

	system_gclk_gen_disable(GCLK_GENERATOR_3);
	system_clock_source_disable(SYSTEM_CLOCK_SOURCE_DFLL);

	system_gclk_gen_disable(GCLK_GENERATOR_1);
}

static void main_clock_select_dfll(void)
{
	struct system_gclk_gen_config gclk_conf;
	struct system_gclk_chan_config dfll_gclk_chan_conf;
	struct system_clock_source_dfll_config dfll_conf;

	/* Select XOSC32K for GCLK1. */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	gclk_conf.run_in_standby = false;
	system_gclk_gen_set_config(GCLK_GENERATOR_1, &gclk_conf);
	system_gclk_gen_enable(GCLK_GENERATOR_1);
	
	system_gclk_chan_get_config_defaults(&dfll_gclk_chan_conf);
	dfll_gclk_chan_conf.source_generator = CONF_CLOCK_DFLL_SOURCE_GCLK_GENERATOR;
	system_gclk_chan_set_config(OSCCTRL_GCLK_ID_DFLL48, &dfll_gclk_chan_conf);
	system_gclk_chan_enable(OSCCTRL_GCLK_ID_DFLL48);

	system_clock_source_dfll_get_config_defaults(&dfll_conf);
	dfll_conf.loop_mode      = CONF_CLOCK_DFLL_LOOP_MODE;
	dfll_conf.on_demand      = 0;
	dfll_conf.run_in_stanby  = CONF_CLOCK_DFLL_RUN_IN_STANDBY;
	dfll_conf.multiply_factor = CONF_CLOCK_DFLL_MULTIPLY_FACTOR;
	dfll_conf.quick_lock = CONF_CLOCK_DFLL_QUICK_LOCK;
	dfll_conf.stable_tracking = SYSTEM_CLOCK_DFLL_STABLE_TRACKING_TRACK_AFTER_LOCK;
	dfll_conf.chill_cycle = SYSTEM_CLOCK_DFLL_CHILL_CYCLE_ENABLE;
	system_clock_source_dfll_set_config(&dfll_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DFLL);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_DFLL));
	if (CONF_CLOCK_DFLL_ON_DEMAND) {
		OSCCTRL->DFLLCTRL.bit.ONDEMAND = 1;
	}

	/* Enable CLK3 for USB */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	system_gclk_gen_set_config(GCLK_GENERATOR_3, &gclk_conf);
	system_gclk_gen_enable(GCLK_GENERATOR_3);

	/* Select DFLL for mainclock. */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	gclk_conf.division_factor = 3;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);
}

static void daq_start(int32_t acq_time)
{
	uint16_t i;
	
	for (i = 0; i < SPECTRUM_LEN; i++)
	{
		gammaSpectrum[i] = 0;
		gammaSpectrumCoincidence[i] = 0;
	}
	gammaPulseN = 0;
	gammaPulseCoincidenceN = 0;
	gammaPulseTstart = rtc_count_get_count(&rtc_instance_app);

	gammaFifoHead = 1;
	gammaFifoTail = 0;

	if (acq_time == 0)
	{
		// Continuous acquisition
		gammaPulseTstop = 0xFFFFFFFF;
	}
	else if (acq_time > 0)
	{
		// Timed acquisition. Configure RTC compare
		gammaPulseTstop = gammaPulseTstart + acq_time;
	}

	rtc_count_set_compare(&rtc_instance_app, gammaPulseTstop, 0);

	gammaTime = 0;
	gammaSum = 0;

	gammaDaqRun = true;

}

static void pomelo_off()
{
	struct port_config pin_conf;

	if (daq_enabled)
	{
		// Make DAC pin input to not drive HV high
		port_get_config_defaults(&pin_conf);
		pin_conf.direction  = PORT_PIN_DIR_INPUT;
		pin_conf.input_pull = PORT_PIN_PULL_UP;

		daq_stop();
		daq_enabled = false;
		coreParams.sys_power = daq_enabled;
		dac_chan_write(&dac_instance_app, DAC_CHANNEL_1, 4095);	// Minimize the chance that spurious signals will trigger readout
		hv_disable();
		dac_disable(&dac_instance_app);
		dac_chan_disable(&dac_instance_app, DAC_CHANNEL_0);
		dac_chan_disable(&dac_instance_app, DAC_CHANNEL_1);
		dac_reset(&dac_instance_app);
		port_pin_set_config(PIN_PA02, &pin_conf);
		port_pin_set_output_level(PIN_PA14, 0);

		port_pin_set_output_level(PIN_PA15, 1);
		nop(); nop(); nop(); nop(); nop(); // 5us. Aaaactually almost nothing
		port_pin_set_output_level(PIN_PA15, 0);
	}
}

static void pomelo_on()
{
	if (daq_enabled)
	{
		// Detector already running. Just restart DAQ
		daq_start(0);
	}
	else
	{
		// Detector off. Switch on
		dac_init_gamma_hv();
		dac_chan_write(&dac_instance_app, DAC_CHANNEL_0, 4095);	// Immediately set a value that drives HV as low as possible
		dac_chan_write(&dac_instance_app, DAC_CHANNEL_1, coreParams.threshold);
		update_hv_temp(true);
		port_pin_set_output_level(PIN_PA14, 1);
		hv_enable();
		daq_enabled = true;
		coreParams.sys_power = daq_enabled;
		daq_start(0);
	}
}

static void load_parameters(void)
{
	uint16_t readBytes;
	uint8_t i;
	
	// Load structure from NVM
	readBytes = sizeof(coreParams);

	for (i = 0; i < readBytes/NVMCTRL_PAGE_SIZE; i++)
	{
		// Read each page at corresponding position
		nvm_read_buffer(NVM_RWW + i*NVMCTRL_PAGE_SIZE, (uint8_t*)(&coreParams) + i*NVMCTRL_PAGE_SIZE, NVMCTRL_PAGE_SIZE);
	}

	if (readBytes > i*NVMCTRL_PAGE_SIZE)
	{
		// We need to read a bit more, but not a full page
		nvm_read_buffer(NVM_RWW + i*NVMCTRL_PAGE_SIZE, (uint8_t*)(&coreParams) + i*NVMCTRL_PAGE_SIZE, readBytes - i*NVMCTRL_PAGE_SIZE);
	}
	
	if (coreParams.version == 0xFF)
	{
		// EEPROM uninitialized, place safe values in structure
		coreParams.vDac[0] = 0.0;
		coreParams.vDac[1] = 1.0;
		coreParams.iMeas[0] = 0.0;
		coreParams.iMeas[1] = 0.0;
		coreParams.threshold = 4095;
		coreParams.sys_power = false;
		coreParams.sys_outputs = 0;			// There is a variable that this has to be synched up with
		coreParams.sys_coincidence = false;   // There is a variable that this has to be synched up with
		coreParams.sys_pulseChar = 0xFF;
	}

	// Load structure from I2C eeprom
	readBytes = sizeof(physicsParams);
	read_physics(readBytes, (uint8_t*)(&physicsParams));

	if (physicsParams.version == 0xFF)
	{
		physicsParams.sipm_vMin = 1;
		physicsParams.sipm_vMax = 4095;
		physicsParams.sipm_v0deg = 4095.0;
		physicsParams.sipm_vTempComp = 0.0;
		physicsParams.ecal[0] = 0;
		physicsParams.ecal[1] = 1;
		physicsParams.ecal[2] = 0;
		physicsParams.uSvph_constant = 0;
		strcpy(physicsParams.detString, "Uninitialized");
		physicsParams.tempType = 0;
	}
}

static int8_t save_parameters(void)
{
	uint16_t writeBytes;
	uint8_t i;
	enum status_code sc;

	coreParams.sys_coincidence = gammaCoincidence;
	coreParams.sys_outputs = listOut;
	coreParams.sys_power = daq_enabled;
	coreParams.sys_pulseChar = gammaPulseChar;
	coreParams.version = 0x01;

	writeBytes = sizeof(coreParams);
	
	// Erase enough rows to make room, even if we overerase
	for (i = 0; i < writeBytes/(NVMCTRL_PAGE_SIZE*NVMCTRL_ROW_PAGES) + 1; i++)
	{
		sc = nvm_erase_row(NVM_RWW + i * NVMCTRL_PAGE_SIZE * NVMCTRL_ROW_PAGES);
		if (sc != STATUS_OK) return -1;
	}

	// Write structure to NVM
	for (i = 0; i < writeBytes/NVMCTRL_PAGE_SIZE; i++)
	{
		// Write each page at corresponding position
		sc = nvm_write_buffer(NVM_RWW + i*NVMCTRL_PAGE_SIZE, (uint8_t*)(&coreParams) + i*NVMCTRL_PAGE_SIZE, NVMCTRL_PAGE_SIZE);
		if (sc != STATUS_OK) return -2;
	}
	
	if (writeBytes > i*NVMCTRL_PAGE_SIZE)
	{
		// We need to write a bit more, but not a full page
		sc = nvm_write_buffer(NVM_RWW + i*NVMCTRL_PAGE_SIZE, (uint8_t*)(&coreParams) + i*NVMCTRL_PAGE_SIZE, writeBytes - i*NVMCTRL_PAGE_SIZE);
		if (sc != STATUS_OK) return -2;
	}

	physicsParams.version = 0x01;
	writeBytes = sizeof(physicsParams);

	// Write structure to I2C eeprom
	for (i = 0; i < writeBytes/16; i++)
	{
		write_physics(i, 16, (uint8_t*)(&physicsParams) + i*16);
	}

	if (writeBytes > i*16)
	{
		// We need to write a bit more, but not a full page
		write_physics(i, writeBytes - i*16, (uint8_t*)(&physicsParams) + i*16);
	}

	return 0;
}

static void upgradePhysics()
{
	struct physics_params_old physicsParamsOld;
	uint16_t readBytes;

	// Load structure from I2C eeprom
	readBytes = sizeof(physicsParamsOld);
	read_physics(readBytes, (uint8_t*)(&physicsParamsOld));

	if (physicsParamsOld.initialized == 0x55)
	{
		physicsParams.version = 0x01;
		physicsParams.sipm_vMin = physicsParamsOld.sipm_vMin;
		physicsParams.sipm_vMax = physicsParamsOld.sipm_vMax;
		physicsParams.sipm_v0deg = physicsParamsOld.sipm_v0deg;
		physicsParams.sipm_vTempComp = physicsParamsOld.sipm_vTempComp;
		physicsParams.ecal[0] = physicsParamsOld.ecal[0];
		physicsParams.ecal[1] = physicsParamsOld.ecal[1];
		physicsParams.ecal[2] = physicsParamsOld.ecal[2];
		physicsParams.uSvph_constant = physicsParamsOld.uSvph_constant;
		strcpy(physicsParams.detString, physicsParamsOld.detString);
		physicsParams.tempType = physicsParamsOld.tempType;
	}
}

static void upgradeCore()
{
	struct core_params_old coreParamsOld;
	uint16_t readBytes;
	uint8_t i;
	
	// Load structure from NVM
	readBytes = sizeof(coreParamsOld);

	for (i = 0; i < readBytes/NVMCTRL_PAGE_SIZE; i++)
	{
		// Read each page at corresponding position
		nvm_read_buffer(NVM_RWW + i*NVMCTRL_PAGE_SIZE, (uint8_t*)(&coreParamsOld) + i*NVMCTRL_PAGE_SIZE, NVMCTRL_PAGE_SIZE);
	}

	if (readBytes > i*NVMCTRL_PAGE_SIZE)
	{
		// We need to read a bit more, but not a full page
		nvm_read_buffer(NVM_RWW + i*NVMCTRL_PAGE_SIZE, (uint8_t*)(&coreParamsOld) + i*NVMCTRL_PAGE_SIZE, readBytes - i*NVMCTRL_PAGE_SIZE);
	}
	
	if (coreParamsOld.initialized == 0x55)
	{
		gammaCoincidence = coreParamsOld.sys_coincidence;
		listOut = coreParamsOld.sys_outputs;
		daq_enabled = coreParamsOld.sys_power;
		gammaPulseChar = coreParamsOld.sys_pulseChar;
		coreParams.version = 0x01;
		coreParams.iMeas[0] = 0;
		coreParams.iMeas[1] = 0;
		coreParams.vDac[0] = coreParamsOld.vdac[0];
		coreParams.vDac[1] = coreParamsOld.vdac[1];
		coreParams.threshold = coreParamsOld.threshold;
		coreParams.sys_power = coreParamsOld.sys_power;
		coreParams.sys_outputs = coreParamsOld.sys_outputs;
		coreParams.sys_coincidence = coreParamsOld.sys_coincidence;
		coreParams.sys_pulseChar = coreParamsOld.sys_pulseChar;
	}

}

static int8_t clear_parameters(void)
{
	uint16_t writeBytes;
	uint8_t i;
	enum status_code sc;

	writeBytes = sizeof(coreParams);
	
	// Erase enough rows to make room, even if we overerase
	for (i = 0; i < writeBytes/(NVMCTRL_PAGE_SIZE*NVMCTRL_ROW_PAGES) + 1; i++)
	{
		sc = nvm_erase_row(NVM_RWW + i * NVMCTRL_PAGE_SIZE * NVMCTRL_ROW_PAGES);
		if (sc != STATUS_OK) return -1;
	}
	return 0;
}



static void pomelo_dosimetry(float *cpm, float *uSvph, float *seconds)
{
	uint32_t l_gammaTime;
	uint32_t l_gammaCounts;
	uint32_t l_gammaSum;
	uint64_t l_gammaSumSquare;
	float eAdc0, eAdc1, eAdc2;
	
	cpu_irq_disable();
	timer_updateValues_unsafe();
	l_gammaTime = gammaTime;
	l_gammaCounts = gammaCounts;
	l_gammaSum = gammaSum;
	l_gammaSumSquare = gammaSumSquare;
	gammaTime = 0;
	gammaCounts = 0;
	gammaSum = 0;
	gammaSumSquare = 0;
	cpu_irq_enable();
	
	if (l_gammaTime == 0)
	{
		*cpm = 0;
		*uSvph = 0;
		*seconds = 0;
	}
	else
	{
		// N * 60 / t[s]
		// N * 60 * 32768 / t[clocks]
		*cpm = l_gammaCounts * 60.0 * 32768.0;
		*cpm /= l_gammaTime;

		eAdc0 = physicsParams.ecal[0] * l_gammaCounts;
		eAdc1 = physicsParams.ecal[1] * l_gammaSum;
		eAdc2 = physicsParams.ecal[2] * l_gammaSumSquare;

		// E[ADC] * 639.45 * 1e-4 / t[ms]
		// E[ADC] * 639.45 * 1e-7 / t[s]
		// E[ADC] * 639.45 * 32768 * 1e-7 / t[clocks]
		*uSvph = (eAdc0 + eAdc1 + eAdc2) * 2.09534976;
		*uSvph /= l_gammaTime;
		
		*seconds = l_gammaTime / 32768.0;
	}
}



static void param_set(uint8_t iFace, uint8_t *buf)
{
	int32_t parameter;
	float setting;
	char *separatorPtr, *endPtr;
	
	parameter = strtol(buf, &separatorPtr, 10);
	if (buf == separatorPtr)
	{
		// Malformed input. No parameter given
		command_response(iFace, -2);
		return;
	}
	
	if (separatorPtr[0] != ':')
	{
		// Malformed input
		command_response(iFace, -2);
		return;
	}

	if (separatorPtr[1] == 0)
	{
		// Malformed input -- missing value
		command_response(iFace, -2);
		return;
	}

	setting = strtof(&separatorPtr[1], &endPtr);
	
	if (*endPtr != 0)
	{
		// Malformed input -- extra stuff at the end
		command_response(iFace, -2);
		return;
	}

	// Ok, place setting in parameter.
	switch(parameter)
	{
		case 0:
			// SiPM vmin
			if (setting < 0 || setting > 4096) { command_response(iFace, -1); return; }
			physicsParams.sipm_vMin = setting;
			if (daq_enabled) update_hv_temp(false);
			break;
		case 1:
			if (setting < 0 || setting > 4096) { command_response(iFace, -1); return; }
			physicsParams.sipm_vMax = setting;
			if (daq_enabled) update_hv_temp(false);
			break;
		case 2:
			if (setting < 0 || setting > 4096) { command_response(iFace, -1); return; }
			physicsParams.sipm_v0deg = setting;
			if (daq_enabled) update_hv_temp(false);
			break;
		case 3:
			if (setting < -5 || setting > 5) { command_response(iFace, -1); return; }
			physicsParams.sipm_vTempComp = setting;
			if (daq_enabled) update_hv_temp(false);
			break;
		case 4:
			physicsParams.ecal[0] = setting;
			break;
		case 5:
			physicsParams.ecal[1] = setting;
			break;
		case 6:
			physicsParams.ecal[2] = setting;
			break;
		case 7:
			physicsParams.uSvph_constant = setting;
			break;
		case 8:
			coreParams.vDac[0] = setting;
			if (daq_enabled) update_hv_temp(false);
			break;
		case 9:
			coreParams.vDac[1] = setting;
			if (daq_enabled) update_hv_temp(false);
			break;
		case 10:
			coreParams.iMeas[0] = setting;
			break;
		case 11:
			coreParams.iMeas[1] = setting;
			break;
		case 12:
			coreParams.iMeas[2] = setting;
			break;
		case 13:
			if (setting < 1 || setting > 4096) { command_response(iFace, -1); return; }
			coreParams.threshold = setting;
			dac_chan_write(&dac_instance_app, DAC_CHANNEL_1, coreParams.threshold);
			break;
		case 14:
			if (setting < 0 || setting > 127) { command_response(iFace, -1); return; }
			gammaFifoHead = 1;
			gammaFifoTail = 0;
			listOut = (int16_t)setting;
			coreParams.sys_outputs = listOut;
			break;
		case 15:
			if (setting < 0 || setting > 1) { command_response(iFace, -1); return; }
			if ((int16_t)setting == 0)
			{
				coincidences_reset();
				gammaCoincidence = false;
			}
			else
			{
				gammaCoincidence = true;
				coincidences_enable();
			}
			coreParams.sys_coincidence = gammaCoincidence;
			break;
		case 16:
			if (setting < 128 || setting > 255) { command_response(iFace, -1); return; }
			gammaPulseChar = (uint8_t)setting;
			coreParams.sys_pulseChar = gammaPulseChar;
			break;
		case 100:
			if ((int16_t)setting != -2024) { command_response(iFace, -1); return; }
			command_response(iFace, save_parameters());
			return;
			break;
		case 200:
			if ((int16_t)setting != -2024) { command_response(iFace, -1); return; }
			pomelo_off();
			adc_calib(iFace);
			break;
		case 300:
			if ((int16_t)setting != -2024) { command_response(iFace, -1); return; }
			init_physics();
			break;
		case 1000:
			if ((int16_t)setting != -2024) { command_response(iFace, -1); return; }
			pomelo_off();
			NVIC_SystemReset();
			break;
		case 2000:
			if ((int16_t)setting != -2024) { command_response(iFace, -1); return; }
			pomelo_off();
			reset_to_bootloader();
			break;
		default:
			command_response(iFace, -2);
			return;
	}


	command_response(iFace, 0);
}



static void command_data_handler(uint8_t iFace, uint8_t rxChar)
{
	static int16_t count = 0;
	static uint32_t i;
	static float temp, cpm, usvph;
	static uint8_t paramBuf[64];

	switch (cmdState)
	{
		case STATE_CMD_IDLE:
			switch (rxChar)
			{
				case 'h':
					// Histogram
					pomelo_printf(iFace, "{\"type\":\"spectrum\", \"payload\":{");
					pomelo_sprintf(iFace, "\"sn\":\"%08X%08X%08X%08X\",", *(uint32_t*)0x0080A00C, *(uint32_t*)0x0080A040, *(uint32_t*)0x0080A044, *(uint32_t*)0x0080A048);
					pomelo_sprintf(iFace, "\"threshold\":%d,", coreParams.threshold);
					pomelo_sprintf(iFace, "\"count\":%lu,", gammaPulseN);
					pomelo_sprintf(iFace, "\"ecal\":[%8e,%8e,%8e],", physicsParams.ecal[0], physicsParams.ecal[1], physicsParams.ecal[2]);
					if (gammaCoincidence)
					{
						pomelo_sprintf(iFace, "\"coincidenceCount\":%lu,", gammaPulseCoincidenceN);
					}
					temp = read_temp();
					pomelo_sprintf(iFace, "\"temperature\":%.5f,", temp);
					if (gammaDaqRun)
					{
						pomelo_sprintf(iFace, "\"time\":%lu,", rtc_count_get_count(&rtc_instance_app) - gammaPulseTstart);
					}
					else
					{
						pomelo_sprintf(iFace, "\"time\":%lu,", gammaPulseTstop - gammaPulseTstart);
					}
					pomelo_printf(iFace, "\"data\":[");
					for (i = 0; i < SPECTRUM_LEN - 1; i++)
					{
						pomelo_sprintf(iFace, "%lu,", (unsigned long)gammaSpectrum[i]);
					}
					pomelo_sprintf(iFace, "%lu]", (unsigned long)gammaSpectrum[SPECTRUM_LEN - 1]);
					if (gammaCoincidence)
					{
						pomelo_printf(iFace, ",\"coincidence\":[");
						for (i = 0; i < SPECTRUM_LEN - 1; i++)
						{
							pomelo_sprintf(iFace, "%lu,", (unsigned long)gammaSpectrumCoincidence[i]);
						}
						pomelo_sprintf(iFace, "%lu]", (unsigned long)gammaSpectrumCoincidence[SPECTRUM_LEN - 1]);
					}
					pomelo_printf(iFace, "}}\n");
					break;
				case 's':
					// System
					pomelo_printf(iFace, "{\"type\":\"system\", \"payload\":{");
					pomelo_sprintf(iFace, "\"sn\":\"%08X%08X%08X%08X\",", *(uint32_t*)0x0080A00C, *(uint32_t*)0x0080A040, *(uint32_t*)0x0080A044, *(uint32_t*)0x0080A048);
					pomelo_sprintf(iFace, "\"uptime\":%lu,", rtc_count_get_count(&rtc_instance_app));
					pomelo_sprintf(iFace, "\"running\":%d,", coreParams.sys_power);
					pomelo_sprintf(iFace, "\"temperature\":%.5f", read_temp());
					pomelo_printf(iFace, "}}\n");
					break;
				case 'c':
					// Config
					pomelo_printf(iFace, "{\"type\":\"config\", \"payload\":{\"sipm\":{");
					pomelo_sprintf(iFace, "\"vMin\":%f,", physicsParams.sipm_vMin);
					pomelo_sprintf(iFace, "\"vMax\":%f,", physicsParams.sipm_vMax);
					pomelo_sprintf(iFace, "\"v0deg\":%f,", physicsParams.sipm_v0deg);
					pomelo_sprintf(iFace, "\"vTempComp\":%f},", physicsParams.sipm_vTempComp);
					pomelo_sprintf(iFace, "\"vdac\":[%f,%f],", coreParams.vDac[0], coreParams.vDac[1]);
					pomelo_sprintf(iFace, "\"ecal\":[%8e,%8e,%8e],", physicsParams.ecal[0], physicsParams.ecal[1], physicsParams.ecal[2]);
					pomelo_sprintf(iFace, "\"threshold\":%d,", coreParams.threshold);
					pomelo_sprintf(iFace, "\"coincidence\":%d,", coreParams.sys_coincidence);
					pomelo_sprintf(iFace, "\"outputs\":{\"uart\":{\"pulse\":%d,\"fastPulse\":%d,\"adc\":%d,\"adcTs\":%d},",
						((coreParams.sys_outputs >> LIST_UART_PULSE) & 0x01),
						((coreParams.sys_outputs >> LIST_UART_FAST_PULSE) & 0x01),
						((coreParams.sys_outputs >> LIST_UART_ENERGY) & 0x01),
						((coreParams.sys_outputs >> LIST_UART_ENERGY_TS) & 0x01));
					pomelo_sprintf(iFace, "\"usb\":{\"pulse\":%d,\"adc\":%d,\"adcTs\":%d},",
						((coreParams.sys_outputs >> LIST_USB_PULSE) & 0x01),
						((coreParams.sys_outputs >> LIST_USB_ENERGY) & 0x01),
						((coreParams.sys_outputs >> LIST_USB_ENERGY_TS) & 0x01));
					pomelo_sprintf(iFace, "\"pulseChar\":%d}", coreParams.sys_pulseChar);
					pomelo_printf(iFace, "}}\n");
					break;
				case 'x':
					pomelo_on();
					command_response(iFace, 0);
					break;
				case 'z':
					pomelo_off();
					command_response(iFace, 0);
					break;
				case 'q':
					// Print pomeloParams structure
					// To be removed once a tool is available for setting parameters
					pomelo_printf(iFace, "\npomeloParams = {\n");
					pomelo_sprintf(iFace, "\t[0] sipm_vMin = %f,\n", physicsParams.sipm_vMin);
					pomelo_sprintf(iFace, "\t[1] sipm_vMax = %f,\n", physicsParams.sipm_vMax);
					pomelo_sprintf(iFace, "\t[2] sipm_v0deg = %f,\n", physicsParams.sipm_v0deg);
					pomelo_sprintf(iFace, "\t[3] sipm_vTempComp = %f,\n", physicsParams.sipm_vTempComp);
					pomelo_sprintf(iFace, "\t[4-6] ecal[3] = {%f, %f, %f},\n", physicsParams.ecal[0], physicsParams.ecal[1], physicsParams.ecal[2]);
					pomelo_sprintf(iFace, "\t[7] uSvph_constant = %f,\n\n", physicsParams.uSvph_constant);
					pomelo_sprintf(iFace, "\t[8-9] vDac[2] = {%f, %f},\n", coreParams.vDac[0], coreParams.vDac[1]);
					pomelo_sprintf(iFace, "\t[10-12] iMeas[3] = {%f, %f, %f},\n", coreParams.iMeas[0], coreParams.iMeas[1], coreParams.iMeas[2]);
					pomelo_sprintf(iFace, "\t[13] threshold = %d,\n", coreParams.threshold);
					pomelo_sprintf(iFace, "\t[14] sys_outputs = %d,\n", coreParams.sys_outputs);
					pomelo_sprintf(iFace, "\t[15] sys_coincidence = %d,\n", coreParams.sys_coincidence);
					pomelo_sprintf(iFace, "\t[16] sys_pulseChar = %d,\n", coreParams.sys_pulseChar);
					pomelo_sprintf(iFace, "\t[x/z] sys_power = %d,\n", coreParams.sys_power);
					pomelo_sprintf(iFace, "\tdetString = %s,\n", physicsParams.detString);
					pomelo_sprintf(iFace, "\ttempType = %d,\n", physicsParams.tempType);
					pomelo_sprintf(iFace, "\tcoreVersion = %d,\n", coreParams.version);
					pomelo_sprintf(iFace, "\tphysicsVersion = %d,\n", physicsParams.version);
					pomelo_printf(iFace, "\n};\n");
					break;
				case 'r':
					load_parameters();
					command_response(iFace, 0);
					break;
				case 'k':
					//upgradePhysics();
					break;
				case 'f':
					//upgradeCore();
					break;
				case 'g':
					// CPM
					pomelo_dosimetry(&cpm, &usvph, &temp);
					pomelo_sprintf(iFace, "%f\n", cpm);
					break;
				case 'u':
					// uSv/h
					pomelo_dosimetry(&cpm, &usvph, &temp);
					pomelo_sprintf(iFace, "%f\n", usvph);
					break;
				case 'm':
					// Measure both CPM and uSv/h
					pomelo_dosimetry(&cpm, &usvph, &temp);
					pomelo_sprintf(iFace, "{\"type\":\"dosimetry\",\"payload\":{\"cpm\":%f,", cpm);
					pomelo_sprintf(iFace, "\"uSv/h\":%f,\"time\":%f}}\n", usvph, temp);
					break;
				case 'i':
					// Read sipm current
					pomelo_sprintf(iFace, "{\"type\":\"current\",\"payload\":{\"uA\":%f,", hvload_uA());
					pomelo_sprintf(iFace, "\"tOff\":%d,\"boost\":%d}}\n", hvloadOff, hvBoost);
					break;
				case 'p':
					// Pomelo parametrized procedure
					cmdState = STATE_CMD_PARAM;
					count = 0;
					break;
				case '/':
					// Boost SiPM power
					//hv_changeBoost(true);
					break;
				case '*':
					// Disable SiPM power boost
					//hv_changeBoost(false);
					break;
				default:
					//command_response(iFace, -128);
					break;
			}
			break;
		case STATE_CMD_PARAM:
			if ((rxChar == '\n') || (rxChar == '\r'))
			{
				// Done, parse it
				paramBuf[count] = 0;
				param_set(iFace, paramBuf);
				cmdState = STATE_CMD_IDLE;
			}
			else if (count == (sizeof(paramBuf) - 1))
			{
				// Overflow, ignore
				command_response(iFace, -2);
				cmdState = STATE_CMD_IDLE;
			}
			else
			{
				// Keep adding characters to the buffer
				paramBuf[count] = rxChar;
				count++;
			}
			break;
		default:
			cmdState = STATE_CMD_IDLE;
			break;
	}
}


static void uart_data_handler(void)
{
	static uint8_t c;
	while (uart_available())
	{
		c = uart_rx();
		command_data_handler(1, c);
	}
}


static void usb_data_handler(void)
{
	if (udi_cdc_is_rx_ready())
	{
		command_data_handler(0, udi_cdc_getc());
	}
}


static void power_config(void)
{
	struct system_voltage_regulator_config system_voltage_regulator_cfg;
	system_voltage_regulator_get_config_defaults(&system_voltage_regulator_cfg);
	system_voltage_regulator_cfg.voltage_scale_period = 0;
	system_voltage_regulator_cfg.voltage_scale_step = 15;
	system_voltage_regulator_set_config(&system_voltage_regulator_cfg);
	system_voltage_regulator_enable();
	
	/*
	nvm_get_config_defaults(&nvm_cfg);
	nvm_cfg.sleep_power_mode = NVM_SLEEP_POWER_MODE_WAKEUPINSTANT;
	nvm_cfg.wait_states = 1;
	nvm_cfg.disable_cache = false;
	nvm_cfg.cache_readmode = NVM_CACHE_READMODE_NO_MISS_PENALTY;
	nvm_set_config(&nvm_cfg);
	*/
}

static void configure_nvm(void)
{
	struct nvm_config config_nvm;

	nvm_get_config_defaults(&config_nvm);
	config_nvm.sleep_power_mode = NVM_SLEEP_POWER_MODE_WAKEUPINSTANT;
	config_nvm.disable_cache = false;
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
}

static void apply_parameters(void)
{
	// copy parameters in struct to variables used in ISR
	gammaCoincidence = coreParams.sys_coincidence;
	listOut = coreParams.sys_outputs;
	gammaPulseChar = coreParams.sys_pulseChar;
	
	// and apply to hardware
	dac_chan_write(&dac_instance_app, DAC_CHANNEL_1, coreParams.threshold);
	
	if (coreParams.sys_coincidence)
	{
		coincidences_enable();
	}
	
	if (coreParams.sys_power)
	{
		pomelo_on();
	}
	else
	{
		pomelo_off();
	}
}



int main(void)
{
	system_init();

	power_config();		// ?? No idea if this actually does anything
	
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	
	// LED
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA19, &pin_conf);
	port_pin_set_output_level(PIN_PA19, 1);

	// Reset cap
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA08, &pin_conf);
	port_pin_set_output_level(PIN_PA08, 0);

	// USBVBUS
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(PIN_PA27, &pin_conf);

	// AFE_EN
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA14, &pin_conf);
	port_pin_set_output_level(PIN_PA14, 1);

	// PeakDet disable
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA15, &pin_conf);
	port_pin_set_output_level(PIN_PA15, 0);

	// Ramp generator enable
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA03, &pin_conf);
	port_pin_set_output_level(PIN_PA03, 0);

	// Ramp generator trigger
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA11, &pin_conf);
	port_pin_set_output_level(PIN_PA11, 0);

	// Ramp generator input
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_PA04, &pin_conf);

	// Trigger input
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_PA18, &pin_conf);

	// Synchronizer input
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_PB08, &pin_conf);

	// PeakDet EN input (2nd trigger)
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_PB03, &pin_conf);

	// HV crowbar. Active low, to engage it when board power is switched off
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA21, &pin_conf);
	port_pin_set_output_level(PIN_PA21, 1);
	
	// HV load measurement
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(PIN_PB10, &pin_conf);


	// Paranoid, just make sure HV PWM starts off
	hvBoost = false;
	hv_disable();

	// Clear peak detector capacitor
	port_pin_set_output_level(PIN_PA08, 1);
	nop(); nop(); nop(); nop(); nop(); // 5us
	nop(); nop(); nop(); nop(); nop(); // 5us
	nop(); nop(); nop(); nop(); nop(); // 5us
	nop(); nop(); nop(); nop(); nop(); // 5us
	nop(); nop(); nop(); nop(); nop(); // 5us
	nop(); nop(); nop(); nop(); nop(); // 5us
	port_pin_set_output_level(PIN_PA08, 0);

	unused_pins_init();

	// MCHP: host_uart_config.generator_source = GCLK_GENERATOR_3; added to init()

	rtc_init_gamma();
	eic_init();

	configure_nvm();

	isr_vbus = false;
	can_sleep = true;
	daq_enabled = false;
	temp_comp_run = false;

	gammaDaqRun = false;
	gammaCoincidence = false;
	gammaSyncTrig = false;
	listOut = 0;

	hvloadTime = 0;

	// Start with USB enabled. Will be switched off by main loop if required
	usb_connected = true;
	main_clock_select_dfll();
	udc_start();

	uart_init();
	adc_init_gamma();
	coincidences_reset();
	timer_init_gamma();

	// Do this to have a consistent state
	dac_init_gamma_hv();
	dac_chan_write(&dac_instance_app, DAC_CHANNEL_0, 4095);	// Immediately set a value that drives HV as low as possible
	dac_chan_write(&dac_instance_app, DAC_CHANNEL_1, 4095);
	daq_enabled = true;
	pomelo_off();

	i2c_init();

	load_parameters();
	apply_parameters();

	temp_init();


	while(1)
	{
		if (usb_connected)
		{
			usb_data_handler();
		}
		
		uart_data_handler();
		
		if (listOut != 0)
		{
			if (((gammaFifoTail + 1) % LIST_FIFO_LEN) != gammaFifoHead)
			{
				gammaFifoTail++;
				if (gammaFifoTail >= LIST_FIFO_LEN)
				{
					gammaFifoTail = 0;
				}
				if ((listOut & (1<<LIST_UART_PULSE)) != 0) uart_tx(&gammaPulseChar, 1);
				if ((listOut & (1<<LIST_UART_ENERGY)) != 0) pomelo_sprintf(1, "%d\n", gammaFifo[gammaFifoTail]);
				if ((listOut & (1<<LIST_UART_ENERGY_TS)) != 0) pomelo_sprintf(1, "%lu,%d\n", gammaFifoTs[gammaFifoTail], gammaFifo[gammaFifoTail]);
				if ((listOut & (1<<LIST_USB_PULSE)) != 0) pomelo_sprintf(0, "%c", gammaPulseChar);
				if ((listOut & (1<<LIST_USB_ENERGY)) != 0) pomelo_sprintf(0, "%d\n", gammaFifo[gammaFifoTail]);
				if ((listOut & (1<<LIST_USB_ENERGY_TS)) != 0) pomelo_sprintf(0, "%lu,%d\n", gammaFifoTs[gammaFifoTail], gammaFifo[gammaFifoTail]);

				can_sleep = false;
			}
			else
			{
				can_sleep = true;
			}
		}

		if (isr_vbus)
		{
			// Switch on USB and turn off low power mode
			usb_connected = true;
			cmdState = STATE_CMD_IDLE;
			main_clock_select_dfll();
			udc_start();
			isr_vbus = false;
		}

		if ((port_pin_get_input_level(PIN_PA27) == 0) && usb_connected)
		{
			// VBUS line is low, USB unplugged. Switch off USB and enable low power mode
			usb_connected = false;
			cmdState = STATE_CMD_IDLE;
			// MCHP: Add "usb_disable(&usb_device);" at the end of udd_disable() in usb_device_udd.c
			udc_stop();
			main_clock_select_osc16m();
			unused_pins_init();
		}
		
		if (temp_comp_run)
		{
			if (daq_enabled) update_hv_temp(false);
			temp_comp_run = false;
		}
		
		if (gammaSyncTrig)
		{
			port_pin_set_output_level(PIN_PB02, 1);
			nop(); nop(); nop(); nop(); nop(); // 5us. Aaaactually almost nothing
			port_pin_set_output_level(PIN_PB02, 0);
			gammaSyncTrig = false;
		}
		
		if (can_sleep && !usb_connected)
		{
			system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
			system_sleep();
		}
	}
}


void pomelo_printf(uint8_t iFace, const char* str)
{
	if (iFace == 0)
	{
		if (usb_connected)
		{
			udi_cdc_multi_write_buf(1,str,strlen(str));
		}
	}
	else if (iFace == 1)
	{
		uart_tx(str,strlen(str));
	}
}

int pomelo_sprintf(uint8_t iFace, const char * format, ... )
{
	static char str[80];
	static va_list args;

	if (iFace == 0)
	{
		if (usb_connected)
		{
			va_start(args, format);
			vsprintf(str,format, args);
			va_end(args);
			udi_cdc_multi_write_buf(1,str,strlen(str));
		}
	}
	else if (iFace == 1)
	{
		va_start(args, format);
		vsprintf(str,format, args);
		va_end(args);
		uart_tx(str,strlen(str));
	}
	return 0;
}

static void reset_to_bootloader(void)
{
	*DBL_TAP_PTR = DBL_TAP_MAGIC;
	NVIC_SystemReset();
}
