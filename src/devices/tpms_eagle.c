#include "decoder.h"

// II II II II BW PP TT AA CC CC

static int tpms_eagle_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t packet_bits = {0};
    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 80);

    if (packet_bits.bits_per_row[row] < 80) {
        return DECODE_FAIL_SANITY;
    }

    // Checksum verification
    uint8_t b[9] = {0};
    bitbuffer_extract_bytes(&packet_bits, 0, 4, b, 72);
    // int chk = crc8(b, 8, 0, 0);
    // TODO: CHECKSUM

    uint32_t id = (unsigned)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];
    uint8_t battery_flag = b[4] >> 7;
    uint8_t interframe = (b[4] & 0x7F) >> 4;    // mask 0111 1111
    uint8_t wo_state = b[4] & 0xF;              // mask 0000 1111
    uint8_t pressure = b[5];
    uint8_t temperature = b[6];
    uint8_t acceleration = b[7];
    uint8_t checksum = b[8];

    printf("id: %08X, battery: %i, pressure: %i, temperature: %i, acceleration: %i\n",
            id, battery_flag, pressure, temperature, acceleration);

    /* clang-format off */
    data_t *data = data_make(
        "model", "Model", DATA_STRING, "EAGLE TPMS",
        "id", "Id", DATA_FORMAT, "%08X", DATA_INT, (unsigned)id,
        "pressure", "Pressure", DATA_INT, pressure,
        "temperature", "Temperature", DATA_INT, temperature,
        "acceleration", "Acceleration", DATA_INT, acceleration,
        "battery", "Battery flag", DATA_INT, battery_flag,
        "interframe", "Interframe", DATA_INT, interframe,
        "wo_state", "WO State", DATA_INT, wo_state,
        "checksum", "Checksum", DATA_INT, checksum,
        "mic", "Integrity",  DATA_STRING, "CRC",
        NULL
    );
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_eagle_decode() */
static int tpms_eagle_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // full preamble is 55 55 59 99 (inverted: aa aa a9 99)
    uint8_t const preamble_pattern[3] = {0xaa, 0xa9, 0x99}; // 16 bits

    int row;
    unsigned bitpos;
    int ret    = 0;
    int events = 0;

    bitbuffer_invert(bitbuffer);

    for (row = 0; row < bitbuffer->num_rows; ++row) {
        bitpos = 0;
        // Find a preamble with enough bits after it that it could be a complete packet
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos,
                preamble_pattern, 16)) + 96 <=
                bitbuffer->bits_per_row[row]) {
            ret = tpms_eagle_decode(decoder, bitbuffer, row, bitpos + 16);
            if (ret > 0)
                events += ret;
            bitpos += 2;
        }
    }

    return events > 0 ? events : ret;
}

static char *output_fields[] = {
    "model",
    "id",
    "pressure",
    "temperature",
    "acceleration",
    "battery",
    "interframe",
    "wo_state",
    "checksum",
    "mic",
    NULL
};

r_device tpms_eagle = {
    .name        = "Eagle (SinoTrack) TPMS Sensors",
    .modulation  = FSK_PULSE_PCM,
    .short_width = 52,  // 12-13 samples @250k
    .long_width  = 52,  // FSK
    .reset_limit = 150, // Maximum gap size before End Of Message [us].
    .decode_fn   = &tpms_eagle_callback,
    .disabled    = 0,
    .fields      = output_fields,
};
