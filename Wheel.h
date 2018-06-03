#define DeadWidth 100
#define DebounceWidth 10

#define USB_LED_OFF 0
#define USB_LED_ON  1
#define USB_DATA_OUT 2
#define USB_DATA_WRITE 3
#define USB_DATA_IN 4

#define LED_PIN (1<<PB1)

#define abs(x) ((x) > 0 ? (x) : (-x))

const uint16_t timingtable_in[] = { 0,218,342,380,401,414,423,430,436,440,444,447,450,453,455,457,459,460,563,564,566,567,569,571,574,577,580,584,588,594,601,610,623,644,682,1023 };
const uint8_t timingtable_out[] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0 };

static uint16_t LowerDead[] = {512 - DeadWidth / 2, 512 - DebounceWidth - DeadWidth / 2};
static uint16_t UpperDead[] = {512 + DeadWidth / 2, 512 + DebounceWidth + DeadWidth / 2};

volatile uint8_t Debounce = 1;
volatile uint16_t AnalogIn = 128;

volatile uint8_t ticktock = 0;
volatile uint8_t timing = 255;

uint8_t get_timing(uint16_t value);