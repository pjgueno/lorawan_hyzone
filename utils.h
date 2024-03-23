
#ifndef utils_h
#define utils_h

#include <WString.h>

#if defined(ESP32)
#include <HardwareSerial.h>
#endif

constexpr unsigned SMALL_STR = 64-1;
constexpr unsigned MED_STR = 256-1;
constexpr unsigned LARGE_STR = 512-1;
constexpr unsigned XLARGE_STR = 1024-1;

#define RESERVE_STRING(name, size) String name((const char*)nullptr); name.reserve(size)

#define UPDATE_MIN(MIN, SAMPLE) if (SAMPLE < MIN) { MIN = SAMPLE; }
#define UPDATE_MAX(MAX, SAMPLE) if (SAMPLE > MAX) { MAX = SAMPLE; }
#define UPDATE_MIN_MAX(MIN, MAX, SAMPLE) { UPDATE_MIN(MIN, SAMPLE); UPDATE_MAX(MAX, SAMPLE); }

extern String check_display_value(double value, double undef, uint8_t len, uint8_t str_len);
extern String tmpl(const __FlashStringHelper* patt, const String& value);


#if defined(ESP32)
#define serialSDS (Serial1)
#endif

enum class PmSensorCmd {
	Start,
	Stop,
	ContinuousMode
};

extern bool SDS_checksum_valid(const uint8_t (&data)[8]);
extern void SDS_rawcmd(const uint8_t cmd_head1, const uint8_t cmd_head2, const uint8_t cmd_head3);
extern bool SDS_cmd(PmSensorCmd cmd);

#endif
