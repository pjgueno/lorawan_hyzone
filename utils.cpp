/************************************************************************
 *                                                                      *
 *    airRohr firmware                                                  *
 *    Copyright (C) 2016-2020  Code for Stuttgart a.o.                  *
 *    Copyright (C) 2019-2020  Dirk Mueller                             *
 *                                                                      *
 * This program is free software: you can redistribute it and/or modify *
 * it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or    *
 * (at your option) any later version.                                  *
 *                                                                      *
 * This program is distributed in the hope that it will be useful,      *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        *
 * GNU General Public License for more details.                         *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 *                                                                      *
 ************************************************************************
 */

#include <WString.h>
#include "./utils.h"
#include "./defines.h"
#include "./ext_def.h"
#include <cstdint> 
using namespace std;

String tmpl(const __FlashStringHelper* patt, const String& value) {
	String s = patt;
	s.replace("{v}", value);
	return s;
}

/*****************************************************************
 * check display values, return '-' if undefined                 *
 *****************************************************************/
String check_display_value(double value, double undef, uint8_t len, uint8_t str_len) {
	RESERVE_STRING(s, 15);
	s = (value != undef ? String(value) : String("-"));
	while (s.length() < str_len) {
		s = " " + s;
	}
	return s;
}


/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version    *
 *****************************************************************/

template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	return N;
}

//enlevé std::size_t

bool SDS_checksum_valid(const uint8_t (&data)[8]) {
    uint8_t checksum_is = 0;
    for (unsigned i = 0; i < 6; ++i) {
        checksum_is += data[i];
    }
    return (data[7] == 0xAB && checksum_is == data[6]);
}

void SDS_rawcmd(const uint8_t cmd_head1, const uint8_t cmd_head2, const uint8_t cmd_head3) {
	constexpr uint8_t cmd_len = 19;

	uint8_t buf[cmd_len];
	buf[0] = 0xAA;
	buf[1] = 0xB4;
	buf[2] = cmd_head1;
	buf[3] = cmd_head2;
	buf[4] = cmd_head3;
	for (unsigned i = 5; i < 15; ++i) {
		buf[i] = 0x00;
	}
	buf[15] = 0xFF;
	buf[16] = 0xFF;
	buf[17] = cmd_head1 + cmd_head2 + cmd_head3 - 2;
	buf[18] = 0xAB;
	serialSDS.write(buf, cmd_len);
}

bool SDS_cmd(PmSensorCmd cmd) {
	switch (cmd) {
	case PmSensorCmd::Start:
		SDS_rawcmd(0x06, 0x01, 0x01);
		break;
	case PmSensorCmd::Stop:
		SDS_rawcmd(0x06, 0x01, 0x00);
		break;
	case PmSensorCmd::ContinuousMode:
		// TODO: Check mode first before (re-)setting it
		SDS_rawcmd(0x08, 0x01, 0x00);
		SDS_rawcmd(0x02, 0x01, 0x00);
		break;
	}

	return cmd != PmSensorCmd::Stop;
}
