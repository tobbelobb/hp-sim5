#pragma once

#include <cstdint>

constexpr uint32_t PMC_PCK_0 = 0;
constexpr uint32_t PMC_PCK_1 = 1;
constexpr uint32_t PMC_PCK_2 = 2;
constexpr uint32_t PMC_PCK_3 = 3;
constexpr uint32_t PMC_PCK_4 = 4;
constexpr uint32_t PMC_PCK_5 = 5;
constexpr uint32_t PMC_PCK_6 = 6;

inline void pmc_enable_periph_clk(uint32_t) noexcept {}
inline void pmc_disable_periph_clk(uint32_t) noexcept {}
inline void pmc_enable_pck(uint32_t) noexcept {}
inline void pmc_disable_pck(uint32_t) noexcept {}
inline void pmc_enable_upll_clock() noexcept {}
inline void pmc_enable_udpck() noexcept {}
inline void pmc_enable_uhpck() noexcept {}
inline void pmc_enable_sclk_osc_freq_monitor() noexcept {}
inline void pmc_enable_interrupt(uint32_t) noexcept {}
inline void pmc_disable_interrupt(uint32_t) noexcept {}
inline void pmc_switch_pck_to_mck(uint32_t, uint32_t) noexcept {}

inline constexpr uint32_t PMC_PCK_PRES(uint32_t pres) noexcept { return pres; }

