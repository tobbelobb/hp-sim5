const klipperDictRaw = "{\"app\":\"Klipper\",\"build_versions\":\"gcc: (Ubuntu 11.4.0-1ubuntu1~22.04.2) 11.4.0 binutils: (GNU Binutils for Ubuntu) 2.38\",\"commands\":{\"ads1220_attach_load_cell_probe oid=%c load_cell_probe_oid=%c\":85,\"allocate_oids count=%c\":8,\"buttons_ack oid=%c count=%c\":46,\"buttons_add oid=%c pos=%c pin=%u pull_up=%c\":48,\"buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c invert=%c\":47,\"clear_shutdown\":2,\"config_ads1220 oid=%c spi_oid=%c data_ready_pin=%u\":86,\"config_adxl345 oid=%c spi_oid=%c\":69,\"config_analog_in oid=%c pin=%u\":32,\"config_buttons oid=%c button_count=%c\":49,\"config_counter oid=%c pin=%u pull_up=%c\":56,\"config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u\":17,\"config_ds18b20 oid=%c serial=%*s max_error_count=%c\":-19,\"config_endstop oid=%c pin=%c pull_up=%c\":26,\"config_hd44780 oid=%c rs_pin=%u e_pin=%u d4_pin=%u d5_pin=%u d6_pin=%u d7_pin=%u delay_ticks=%u\":62,\"config_hx71x oid=%c gain_channel=%c dout_pin=%u sclk_pin=%u\":82,\"config_i2c oid=%c\":42,\"config_icm20948 oid=%c i2c_oid=%c\":78,\"config_ldc1612 oid=%c i2c_oid=%c\":92,\"config_ldc1612_with_intb oid=%c i2c_oid=%c intb_pin=%c\":91,\"config_lis2dw oid=%c bus_oid=%c bus_oid_type=%c lis_chip_type=%c\":72,\"config_load_cell_probe oid=%c sos_filter_oid=%c\":-25,\"config_mpu9250 oid=%c i2c_oid=%c\":75,\"config_neopixel oid=%c pin=%u data_size=%hu bit_max_ticks=%u reset_min_ticks=%u\":54,\"config_pca9685 oid=%c bus=%c addr=%c channel=%c cycle_ticks=%u value=%hu default_value=%hu max_duration=%u\":-21,\"config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu default_value=%hu max_duration=%u\":45,\"config_reset\":-24,\"config_sos_filter oid=%c max_sections=%c\":-29,\"config_spi oid=%c pin=%u cs_active_high=%c\":38,\"config_spi_angle oid=%c spi_oid=%c spi_angle_type=%c\":95,\"config_spi_shutdown oid=%c spi_oid=%c shutdown_msg=%*s\":33,\"config_spi_without_cs oid=%c\":37,\"config_st7920 oid=%c cs_pin=%u sclk_pin=%u sid_pin=%u sync_delay_ticks=%u cmd_delay_ticks=%u\":59,\"config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u\":23,\"config_thermocouple oid=%c spi_oid=%c thermocouple_type=%c\":66,\"config_tmcuart oid=%c rx_pin=%u pull_up=%c tx_pin=%u bit_time=%u\":51,\"config_trsync oid=%c\":30,\"debug_nop\":9,\"debug_ping data=%*s\":10,\"debug_read order=%c addr=%u\":12,\"debug_write order=%c addr=%u val=%u\":11,\"emergency_stop\":3,\"endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c\":25,\"endstop_query_state oid=%c\":24,\"finalize_config crc=%u\":6,\"get_clock\":5,\"get_config\":7,\"get_uptime\":4,\"hd44780_send_cmds oid=%c cmds=%*s\":61,\"hd44780_send_data oid=%c data=%*s\":60,\"hx71x_attach_load_cell_probe oid=%c load_cell_probe_oid=%c\":81,\"i2c_read oid=%c reg=%*s read_len=%u\":39,\"i2c_set_bus oid=%c i2c_bus=%u rate=%u address=%u\":41,\"i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u\":64,\"i2c_write oid=%c data=%*s\":40,\"identify offset=%u count=%c\":1,\"ldc1612_setup_home oid=%c clock=%u threshold=%u trsync_oid=%c trigger_reason=%c error_reason=%c\":90,\"load_cell_probe_home oid=%c trsync_oid=%c trigger_reason=%c error_reason=%c clock=%u rest_ticks=%u timeout=%u\":-27,\"load_cell_probe_query_state oid=%c\":-28,\"load_cell_probe_set_range oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i trigger_grams=%u grams_per_count=%i\":-26,\"neopixel_send oid=%c\":52,\"neopixel_update oid=%c pos=%hu data=%*s\":53,\"query_ads1220 oid=%c rest_ticks=%u\":84,\"query_ads1220_status oid=%c\":83,\"query_adxl345 oid=%c rest_ticks=%u\":68,\"query_adxl345_status oid=%c\":67,\"query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c\":31,\"query_counter oid=%c clock=%u poll_ticks=%u sample_ticks=%u\":55,\"query_ds18b20 oid=%c clock=%u rest_ticks=%u min_value=%i max_value=%i\":-20,\"query_hx71x oid=%c rest_ticks=%u\":80,\"query_hx71x_status oid=%c\":79,\"query_icm20948 oid=%c rest_ticks=%u\":77,\"query_icm20948_status oid=%c\":76,\"query_ldc1612 oid=%c rest_ticks=%u\":88,\"query_ldc1612_home_state oid=%c\":89,\"query_lis2dw oid=%c rest_ticks=%u\":71,\"query_lis2dw_status oid=%c\":70,\"query_mpu9250 oid=%c rest_ticks=%u\":74,\"query_mpu9250_status oid=%c\":73,\"query_spi_angle oid=%c clock=%u rest_ticks=%u time_shift=%c\":94,\"query_status_ldc1612 oid=%c\":87,\"query_thermocouple oid=%c clock=%u rest_ticks=%u min_value=%u max_value=%u max_invalid_count=%c\":65,\"queue_digital_out oid=%c clock=%u on_ticks=%u\":15,\"queue_pca9685_out oid=%c clock=%u value=%hu\":-22,\"queue_pwm_out oid=%c clock=%u value=%hu\":44,\"queue_step oid=%c interval=%u count=%hu add=%hi\":22,\"reset_step_clock oid=%c clock=%u\":20,\"set_digital_out pin=%u value=%c\":13,\"set_digital_out_pwm_cycle oid=%c cycle_ticks=%u\":16,\"set_next_step_dir oid=%c dir=%c\":21,\"set_pca9685_out bus=%c addr=%c channel=%c cycle_ticks=%u value=%hu\":-23,\"set_pwm_out pin=%u cycle_ticks=%u value=%hu\":43,\"sos_filter_set_active oid=%c n_sections=%c coeff_int_bits=%c\":-32,\"sos_filter_set_section oid=%c section_idx=%c sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i\":-30,\"sos_filter_set_state oid=%c section_idx=%c state0=%i state1=%i\":-31,\"spi_angle_transfer oid=%c data=%*s\":93,\"spi_send oid=%c data=%*s\":34,\"spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u\":36,\"spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u\":63,\"spi_transfer oid=%c data=%*s\":35,\"st7920_send_cmds oid=%c cmds=%*s\":58,\"st7920_send_data oid=%c data=%*s\":57,\"stepper_get_position oid=%c\":19,\"stepper_stop_on_trigger oid=%c trsync_oid=%c\":18,\"tmcuart_send oid=%c write=%*s read=%c\":50,\"trsync_set_timeout oid=%c clock=%u\":28,\"trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c\":29,\"trsync_trigger oid=%c reason=%c\":27,\"update_digital_out oid=%c value=%c\":14},\"config\":{\"ADC_MAX\":4095,\"CLOCK_FREQ\":50000000,\"MCU\":\"linux\",\"PCA9685_MAX\":4096,\"PWM_MAX\":32768,\"STATS_SUMSQ_BASE\":256,\"STEPPER_STEP_BOTH_EDGE\":1},\"enumerations\":{\"bus_oid_type\":{\"i2c\":1,\"spi\":0},\"i2c_bus\":{\"i2c.0\":[0,15]},\"lis_chip_type\":{\"LIS2DW\":0,\"LIS3DH\":1},\"pin\":{\"analog0\":[4096,8],\"gpio0\":[0,288],\"gpiochip0/gpio0\":[0,288],\"gpiochip1/gpio0\":[288,288],\"gpiochip2/gpio0\":[576,288],\"gpiochip3/gpio0\":[864,288],\"gpiochip4/gpio0\":[1152,288],\"gpiochip5/gpio0\":[1440,288],\"gpiochip6/gpio0\":[1728,288],\"gpiochip7/gpio0\":[2016,288],\"gpiochip8/gpio0\":[2304,288],\"pwmchip0/pwm0\":[65536,16],\"pwmchip1/pwm0\":[65552,16],\"pwmchip2/pwm0\":[65568,16],\"pwmchip3/pwm0\":[65584,16],\"pwmchip4/pwm0\":[65600,16],\"pwmchip5/pwm0\":[65616,16],\"pwmchip6/pwm0\":[65632,16],\"pwmchip7/pwm0\":[65648,16]},\"spi_angle_type\":{\"a1333\":0,\"as5047d\":1,\"mt6816\":3,\"mt6826s\":4,\"tle5012b\":2},\"spi_bus\":{\"spidev0.0\":[0,16],\"spidev1.0\":[256,16],\"spidev2.0\":[512,16],\"spidev3.0\":[768,16],\"spidev4.0\":[1024,16],\"spidev5.0\":[1280,16],\"spidev6.0\":[1536,16],\"spidev7.0\":[1792,16]},\"static_string_id\":{\"ADC out of range\":26,\"All PCA9685 channels must have the same cycle_ticks\":63,\"Already finalized\":13,\"Can not set soft pwm cycle ticks while updates pending\":20,\"Can't add signal that is already active\":25,\"Can't assign oid\":11,\"Can't reset time when stepper active\":22,\"Command parser error\":7,\"Command request\":8,\"Could not start DS18B20 reader thread\":84,\"Could not start DS18B20 reader thread (cond init)\":85,\"Could not start DS18B20 reader thread (mutex init)\":86,\"DS18B20 out of range\":81,\"DS18B20 sensor didn't respond in time\":80,\"Error getting monotonic clock time\":83,\"Error on analog read\":72,\"Error reading DS18B20 sensor\":82,\"Filter section index larger than max_sections\":49,\"Force shutdown command\":56,\"GPIO chip device not found\":79,\"HX71x gain/channel out of range 1-4\":46,\"I2C NACK\":31,\"I2C START NACK\":30,\"I2C START READ NACK\":29,\"I2C Timeout\":28,\"Invalid DS18B20 serial id, could not open for reading\":87,\"Invalid DS18B20 serial id, must not contain '/'\":88,\"Invalid buttons retransmit count\":34,\"Invalid command\":5,\"Invalid count parameter\":23,\"Invalid move request size\":14,\"Invalid neopixel data_size\":39,\"Invalid neopixel update command\":38,\"Invalid oid type\":12,\"Invalid pca9685 channel or value\":57,\"Invalid pca9685 value\":59,\"Invalid spi config\":27,\"Invalid spi_angle chip type\":48,\"Invalid thermocouple chip type\":41,\"Max of 8 buttons\":36,\"Message encode error\":6,\"Missed scheduling of next digital out event\":21,\"Missed scheduling of next hard pwm event\":33,\"Missed scheduling of next pca9685 event\":60,\"Move queue overflow\":15,\"Rescheduled timer in the past\":55,\"Safety range reversed\":54,\"Scheduled digital out event will exceed max_duration\":19,\"Scheduled pca9685 event will exceed max_duration\":58,\"Scheduled pwm event will exceed max_duration\":32,\"Set button past maximum button count\":35,\"Shutdown cleared when not shutdown\":2,\"Stepper too far in past\":24,\"Thermocouple reader fault\":40,\"Timer too close\":3,\"Too many i2c devices\":62,\"Too many spi devices\":71,\"Unable to config pwm device\":74,\"Unable to issue spi ioctl\":66,\"Unable to open GPIO chip device\":78,\"Unable to open adc device\":73,\"Unable to open and init PCA9685 device\":61,\"Unable to open i2c device\":75,\"Unable to open in GPIO chip line\":76,\"Unable to open out GPIO chip line\":77,\"Unable to open spi device\":70,\"Unable to set SPI mode\":67,\"Unable to set SPI speed\":68,\"Unable to set non-blocking on spi device\":69,\"Unable to update PCA9685 value\":64,\"Unable to write to spi\":65,\"alloc_chunk failed\":17,\"alloc_chunks failed\":16,\"angle sensor requires cs pin\":47,\"bus_type i2c unsupported\":44,\"bus_type invalid\":43,\"bus_type spi unsupported\":45,\"config_reset only available when shutdown\":9,\"fixed_mul: overflow\":51,\"grams_per_count is invalid\":52,\"model type invalid\":42,\"oids already allocated\":10,\"sentinel timer called\":4,\"sos_filter not property initialized\":50,\"tmcuart data too large\":37,\"trigger_grams too large\":53,\"update_digital_out not valid with active queue\":18},\"thermocouple_type\":{\"MAX31855\":0,\"MAX31856\":1,\"MAX31865\":2,\"MAX6675\":3}},\"license\":\"GNU GPLv3\",\"responses\":{\"analog_in_state oid=%c next_clock=%u value=%hu\":-6,\"buttons_state oid=%c ack_count=%c state=%*s\":-3,\"clock clock=%u\":-13,\"config is_config=%c crc=%u is_shutdown=%c move_count=%hu\":-12,\"counter_state oid=%c next_clock=%u count=%u count_clock=%u\":128,\"debug_result val=%u\":-10,\"ds18b20_result oid=%c next_clock=%u value=%i fault=%u\":135,\"endstop_state oid=%c homing=%c next_clock=%u pin_value=%c\":-8,\"i2c_read_response oid=%c response=%*s\":-4,\"identify_response offset=%u data=%.*s\":0,\"is_shutdown static_string_id=%hu\":-17,\"ldc1612_home_state oid=%c homing=%c trigger_clock=%u\":130,\"load_cell_probe_state oid=%c is_homing_trigger=%c trigger_ticks=%u\":134,\"neopixel_result oid=%c success=%c\":-1,\"pong data=%*s\":-11,\"sensor_bulk_data oid=%c sequence=%hu data=%*s\":133,\"sensor_bulk_status oid=%c clock=%u query_ticks=%u next_sequence=%hu buffered=%u possible_overflows=%hu\":132,\"shutdown clock=%u static_string_id=%hu\":-16,\"spi_angle_transfer_response oid=%c clock=%u response=%*s\":131,\"spi_transfer_response oid=%c response=%*s\":-5,\"starting\":-18,\"stats count=%u sum=%u sumsq=%u\":-15,\"stepper_position oid=%c pos=%i\":-9,\"thermocouple_result oid=%c next_clock=%u value=%u fault=%c\":129,\"tmcuart_response oid=%c read=%*s\":-2,\"trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u\":-7,\"uptime high=%u clock=%u\":-14},\"version\":\"328f27ab-dirty-20250912_144903-torbjorn-MS-7D25\"}";

const baseDictionary = JSON.parse(klipperDictRaw);

export const MESSAGE_MIN = 5;
export const MESSAGE_MAX = 64;
export const MESSAGE_HEADER_SIZE = 2;
export const MESSAGE_TRAILER_SIZE = 3;
export const MESSAGE_TRAILER_SYNC = 1;
export const MESSAGE_TRAILER_CRC = 3;
export const MESSAGE_SEQ_MASK = 0x0f;
export const MESSAGE_DEST = 0x10;
export const MESSAGE_SYNC = 0x7e;

function crc16Ccitt(bytes, length) {
  const len = typeof length === 'number' ? length : bytes.length;
  let crc = 0xffff;
  for (let i = 0; i < len; i += 1) {
    let data = bytes[i];
    data ^= crc & 0xff;
    data ^= (data & 0x0f) << 4;
    crc = (((data << 8) & 0xffff) | ((crc >> 8) & 0xff)) ^ (data >> 4) ^ ((data << 3) & 0xffff);
  }
  return [(crc >> 8) & 0xff, crc & 0xff];
}

function parseVarint(bytes, pos, { signed } = { signed: false }) {
  if (pos >= bytes.length) {
    throw new Error('Truncated varint');
  }
  let c = bytes[pos];
  pos += 1;
  let v = BigInt(c & 0x7f);
  if (signed && (c & 0x60) === 0x60) {
    v |= BigInt(-0x20);
  }
  while (c & 0x80) {
    if (pos >= bytes.length) {
      throw new Error('Truncated varint continuation');
    }
    c = bytes[pos];
    pos += 1;
    v = (v << 7n) | BigInt(c & 0x7f);
  }
  if (signed) {
    return { value: Number(v), pos };
  }
  const mask = 0xffffffffn;
  const u = Number(v & mask);
  return { value: u >>> 0, pos };
}

function encodeInt32(value) {
  const out = [];
  let v = Number(value);
  if (v >= 0xc000000 || v < -0x4000000) {
    out.push(((v >> 28) & 0x7f) | 0x80);
  }
  if (v >= 0x180000 || v < -0x80000) {
    out.push(((v >> 21) & 0x7f) | 0x80);
  }
  if (v >= 0x3000 || v < -0x1000) {
    out.push(((v >> 14) & 0x7f) | 0x80);
  }
  if (v >= 0x60 || v < -0x20) {
    out.push(((v >> 7) & 0x7f) | 0x80);
  }
  out.push(v & 0x7f);
  return out;
}

class UInt32Type {
  constructor() {
    this.isDynamicString = false;
  }
  parse(bytes, pos) {
    return parseVarint(bytes, pos, { signed: false });
  }
  toString(value) {
    return String(value);
  }
}

class Int32Type {
  constructor() {
    this.isDynamicString = false;
  }
  parse(bytes, pos) {
    return parseVarint(bytes, pos, { signed: true });
  }
  toString(value) {
    return String(value);
  }
}

class UInt16Type extends UInt32Type {}
class Int16Type extends Int32Type {}
class ByteType extends UInt32Type {}

class BufferType {
  constructor() {
    this.isDynamicString = true;
    this.decoder = typeof TextDecoder !== 'undefined' ? new TextDecoder('utf-8', { fatal: false }) : null;
  }
  parse(bytes, pos) {
    if (pos >= bytes.length) {
      throw new Error('Truncated buffer length');
    }
    const length = bytes[pos];
    const start = pos + 1;
    const end = start + length;
    if (end > bytes.length) {
      throw new Error('Truncated buffer payload');
    }
    const slice = bytes.subarray(start, end);
    return { value: slice, pos: end };
  }
  toString(value) {
    if (value == null) {
      return '';
    }
    if (this.decoder) {
      try {
        return JSON.stringify(this.decoder.decode(value));
      } catch (_) {
        // fall through to hex formatting
      }
    }
    const hex = Array.from(value, (b) => b.toString(16).padStart(2, '0')).join('');
    return `0x${hex}`;
  }
}

class EnumerationType {
  constructor(innerType, enumName, reverseMap) {
    this.innerType = innerType;
    this.enumName = enumName;
    this.reverseMap = reverseMap;
    this.isDynamicString = innerType.isDynamicString;
  }
  parse(bytes, pos) {
    const { value, pos: nextPos } = this.innerType.parse(bytes, pos);
    const mapped = this.reverseMap.get(value);
    if (mapped !== undefined) {
      return { value: mapped, pos: nextPos };
    }
    return { value: `?${value}`, pos: nextPos };
  }
  toString(value) {
    return String(value);
  }
}

function createBaseType(fmt) {
  switch (fmt) {
    case '%u':
      return new UInt32Type();
    case '%i':
      return new Int32Type();
    case '%hu':
      return new UInt16Type();
    case '%hi':
      return new Int16Type();
    case '%c':
      return new ByteType();
    case '%s':
    case '%.*s':
    case '%*s':
      return new BufferType();
    default:
      throw new Error(`Unsupported format specifier: ${fmt}`);
  }
}

function buildEnumerationReverse(rawEnumerations = {}) {
  const out = new Map();
  for (const [enumName, entries] of Object.entries(rawEnumerations)) {
    const map = new Map();
    for (const [key, value] of Object.entries(entries || {})) {
      if (Array.isArray(value)) {
        const [startValue, count] = value;
        let root = key;
        while (root && /\d$/.test(root)) {
          root = root.slice(0, -1);
        }
        const suffix = key.slice(root.length);
        const startIndex = suffix ? parseInt(suffix, 10) : 0;
        for (let idx = 0; idx < count; idx += 1) {
          map.set(startValue + idx, `${root}${startIndex + idx}`);
        }
      } else {
        map.set(value, key);
      }
    }
    out.set(enumName, map);
  }
  return out;
}

class MessageFormat {
  constructor(msgid, msgformat, type, enumerations) {
    this.msgid = msgid;
    this.msgformat = msgformat;
    this.type = type;
    const parts = msgformat.trim().split(/\s+/);
    this.name = parts[0];
    this.paramDefs = [];
    this.msgidBytes = encodeInt32(msgid);
    const paramParts = parts.slice(1);
    for (const param of paramParts) {
      if (!param.includes('=')) {
        continue;
      }
      const [rawName, fmt] = param.split('=');
      const name = rawName.trim();
      const baseType = createBaseType(fmt.trim());
      let typeInstance = baseType;
      for (const [enumName, reverse] of enumerations) {
        if (name === enumName || name.endsWith(`_${enumName}`)) {
          typeInstance = new EnumerationType(baseType, enumName, reverse);
          break;
        }
      }
      this.paramDefs.push({ name, type: typeInstance });
    }
  }

  parse(bytes, offset) {
    let pos = offset + this.msgidBytes.length;
    const params = {};
    for (const { name, type } of this.paramDefs) {
      const { value, pos: nextPos } = type.parse(bytes, pos);
      params[name] = value;
      pos = nextPos;
    }
    return { params, nextPos: pos };
  }

  format(params) {
    const pieces = [this.name];
    for (const { name, type } of this.paramDefs) {
      const value = params[name];
      pieces.push(`${name}=${type.toString(value)}`);
    }
    return pieces.join(' ');
  }
}

class UnknownFormat {
  constructor() {
    this.name = '#unknown';
  }

  parse(bytes, offset) {
    const payloadEnd = bytes.length - MESSAGE_TRAILER_SIZE;
    return {
      params: { '#msg': bytes.subarray(offset, payloadEnd) },
      nextPos: payloadEnd,
    };
  }

  format(params) {
    const payload = params['#msg'];
    if (!payload) {
      return '#unknown';
    }
    const hex = Array.from(payload, (b) => b.toString(16).padStart(2, '0')).join('');
    return `#unknown 0x${hex}`;
  }
}

export class KlipperSerialDecoder {
  constructor(dictionary = baseDictionary) {
    this.dictionary = dictionary;
    this.enumerations = buildEnumerationReverse(dictionary?.enumerations);
    this.messagesById = new Map();
    this.msgidParser = new Int32Type();
    this.unknown = new UnknownFormat();
    this._initMessages(dictionary?.commands, 'command');
    this._initMessages(dictionary?.responses, 'response');
    this._initMessages(dictionary?.output, 'output');
  }

  _initMessages(defs = {}, type) {
    if (!defs) {
      return;
    }
    for (const [msgformat, msgid] of Object.entries(defs)) {
      const fmt = new MessageFormat(Number(msgid), msgformat, type, this.enumerations);
      this.messagesById.set(Number(msgid), fmt);
    }
  }

  checkPacket(buffer) {
    if (!buffer || buffer.length < MESSAGE_MIN) {
      return 0;
    }
    const msglen = buffer[0];
    if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX) {
      return -1;
    }
    if (buffer.length < msglen) {
      return 0;
    }
    const seqByte = buffer[1];
    if ((seqByte & ~MESSAGE_SEQ_MASK) !== MESSAGE_DEST) {
      return -1;
    }
    if (buffer[msglen - MESSAGE_TRAILER_SYNC] !== MESSAGE_SYNC) {
      return -1;
    }
    const crcIndex = msglen - MESSAGE_TRAILER_CRC;
    const expectedCrcHi = buffer[crcIndex];
    const expectedCrcLo = buffer[crcIndex + 1];
    const [crcHi, crcLo] = crc16Ccitt(buffer.subarray(0, msglen - MESSAGE_TRAILER_SIZE));
    if (crcHi !== expectedCrcHi || crcLo !== expectedCrcLo) {
      return -1;
    }
    return msglen;
  }

  decodePacket(packet) {
    if (!packet || packet.length < MESSAGE_MIN) {
      return { seq: null, lines: [] };
    }
    const msglen = packet[0];
    const seq = packet[1] & MESSAGE_SEQ_MASK;
    const lines = [];
    let pos = MESSAGE_HEADER_SIZE;
    const end = msglen - MESSAGE_TRAILER_SIZE;
    while (pos < end) {
      const { value: msgid } = this.msgidParser.parse(packet, pos);
      const handler = this.messagesById.get(msgid) || this.unknown;
      const { params, nextPos } = handler.parse(packet, pos);
      pos = nextPos;
      const formatted = handler.format(params);
      if (formatted) {
        lines.push(formatted);
      }
      if (pos >= end) {
        break;
      }
    }
    return { seq, lines };
  }
}

let sharedDecoder = null;

function getSharedDecoder() {
  if (!sharedDecoder) {
    sharedDecoder = new KlipperSerialDecoder(baseDictionary);
  }
  return sharedDecoder;
}

function asUint8Array(chunk) {
  if (chunk instanceof Uint8Array) {
    return chunk;
  }
  if (chunk instanceof ArrayBuffer) {
    return new Uint8Array(chunk);
  }
  if (ArrayBuffer.isView(chunk)) {
    return new Uint8Array(chunk.buffer, chunk.byteOffset, chunk.byteLength);
  }
  return null;
}

function concatBuffers(existing, next) {
  if (!existing || existing.length === 0) {
    return next ? next.slice() : new Uint8Array(0);
  }
  if (!next || next.length === 0) {
    return existing;
  }
  const out = new Uint8Array(existing.length + next.length);
  out.set(existing, 0);
  out.set(next, existing.length);
  return out;
}

export class SerialLineDecoder {
  constructor(decoder) {
    this.decoder = decoder || getSharedDecoder();
    this.buffer = new Uint8Array(0);
  }

  push(chunk) {
    const data = asUint8Array(chunk);
    if (!data || data.length === 0) {
      return [];
    }
    this.buffer = concatBuffers(this.buffer, data);
    const lines = [];
    while (true) {
      const len = this.decoder.checkPacket(this.buffer);
      if (len > 0) {
        const packet = this.buffer.subarray(0, len);
        const { lines: newLines } = this.decoder.decodePacket(packet);
        if (newLines.length) {
          lines.push(...newLines);
        }
        this.buffer = this.buffer.subarray(len);
        continue;
      }
      if (len < 0) {
        this.buffer = this.buffer.subarray(1);
        continue;
      }
      break;
    }
    return lines;
  }

  flush() {
    const lines = [];
    while (this.buffer.length >= MESSAGE_MIN) {
      const len = this.decoder.checkPacket(this.buffer);
      if (len > 0) {
        const packet = this.buffer.subarray(0, len);
        const { lines: newLines } = this.decoder.decodePacket(packet);
        if (newLines.length) {
          lines.push(...newLines);
        }
        this.buffer = this.buffer.subarray(len);
      } else if (len < 0) {
        this.buffer = this.buffer.subarray(1);
      } else {
        break;
      }
    }
    this.buffer = new Uint8Array(0);
    return lines;
  }
}

export async function* iterateSerialLines(stream, decoder) {
  const activeDecoder = decoder || getSharedDecoder();
  const reader = stream.getReader();
  const lineDecoder = new SerialLineDecoder(activeDecoder);
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) {
        break;
      }
      const chunk = asUint8Array(value);
      if (!chunk || chunk.length === 0) {
        continue;
      }
      const lines = lineDecoder.push(chunk);
      for (const line of lines) {
        yield line;
      }
    }
  } finally {
    if (reader.releaseLock) {
      reader.releaseLock();
    }
  }
  const remaining = lineDecoder.flush();
  for (const line of remaining) {
    yield line;
  }
}

export function decodeBase64Chunk(b64) {
  if (typeof b64 !== 'string' || !b64) {
    return null;
  }
  if (typeof atob !== 'function') {
    return null;
  }
  const binary = atob(b64);
  const len = binary.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i += 1) {
    bytes[i] = binary.charCodeAt(i);
  }
  return bytes;
}

export function createKlipperSerialDecoder() {
  return new KlipperSerialDecoder(baseDictionary);
}
