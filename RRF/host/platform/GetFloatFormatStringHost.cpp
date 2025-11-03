#include <RepRapFirmware.h>

const char* GetFloatFormatString(float, unsigned int numDigitsAfterPoint) noexcept
{
    static constexpr const char* formats[] = {"%.0f", "%.1f", "%.2f", "%.3f", "%.4f",
                                              "%.5f", "%.6f", "%.7f", "%.8f"};
    constexpr unsigned int maxDigits =
        static_cast<unsigned int>(sizeof(formats) / sizeof(formats[0]) - 1);
    const unsigned int idx =
        (numDigitsAfterPoint > maxDigits) ? maxDigits : numDigitsAfterPoint;
    return formats[idx];
}
