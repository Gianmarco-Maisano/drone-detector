/*
 * Passively detects DJI OcuSync/Lightbridge-like signals in the 2.4 and 5.8 GHz
 * bands. Confidence combines frequency hopping, wide and narrow signal
 * activity, persistence, signal-to-noise ratio, and a penalty for signals
 * resembling fixed 802.11 channels. This is a spectrum heuristic, not a
 * protocol decoder or a reliable drone identifier.
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef _UI_DRONEDETECT
#define _UI_DRONEDETECT

#include "ui.hpp"
#include "ui_receiver.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "portapack.hpp"
#include "baseband_api.hpp"
#include "radio_state.hpp"
#include "receiver_model.hpp"
#include "string_format.hpp"
#include "audio.hpp"
#include "spectrum_color_lut.hpp"

#include <vector>
#include <array>

namespace ui::external_app::dronedetect {

// Each slice captures 20 MHz, represented by 256 power bins (~78 kHz per bin).
#define DRONE_DETECT_SLICE_BW 20000000
#define DRONE_DETECT_NB_BINS 256
#define DRONE_DETECT_WATERFALL_WIDTH 240
#define DRONE_DETECT_BIN_HZ (DRONE_DETECT_SLICE_BW / DRONE_DETECT_NB_BINS)

// Baseband spectrum values are scaled as (dB * 5 + 255): 5 counts per dB.
#define DRONE_DETECT_COUNTS_PER_DB 5

class DroneDetectView : public ui::View {
   public:
    DroneDetectView(ui::NavigationView& nav);
    ~DroneDetectView();

    DroneDetectView(const DroneDetectView&) = delete;
    DroneDetectView& operator=(const DroneDetectView&) = delete;

    std::string title() const override { return "Drone Detect"; };

    void on_show() override;
    void on_hide() override;
    void focus() override;
    void paint(Painter& painter) override;

   private:
    static constexpr uint8_t DEFAULT_MARGIN_DB = 14;
    static constexpr uint8_t DEFAULT_MIN_WIDTH_MHZ = 3;
    static constexpr uint8_t DEFAULT_ALARM_CONF = 70;
    static constexpr uint16_t DEFAULT_HOLD_FRAMES = 15;
    static constexpr uint8_t DEFAULT_BAND_DWELL_SECONDS = 3;
    static constexpr uint32_t SIGNAL_DISPLAY_HOLD_MS = 3000;
    static constexpr size_t CENTER_HISTORY = 8;

    uint8_t margin_db_{DEFAULT_MARGIN_DB};
    uint8_t min_width_mhz_{DEFAULT_MIN_WIDTH_MHZ};
    uint8_t alarm_conf_{DEFAULT_ALARM_CONF};
    uint16_t hold_frames_{DEFAULT_HOLD_FRAMES};
    uint8_t band_dwell_seconds_{DEFAULT_BAND_DWELL_SECONDS};
    uint8_t lna_gain_24_{32};
    uint8_t vga_gain_24_{32};
    uint8_t lna_gain_58_{32};
    uint8_t vga_gain_58_{32};
    uint16_t band_start_mhz_24_{2400};
    uint16_t band_end_mhz_24_{2483};
    uint16_t band_start_mhz_58_{5725};
    uint16_t band_end_mhz_58_{5850};

    struct SliceFeature {
        uint8_t noise{0};
        uint8_t peak{0};
        uint16_t width{0};
        uint16_t center_bin{0};
        bool active{false};
    };

    void build_slices();
    void retune();
    void on_channel_spectrum(const ChannelSpectrum& spectrum);
    void analyze_slice(const ChannelSpectrum& spectrum);
    void end_of_sweep();
    void evaluate_confidence();
    void handle_alarm();
    bool looks_like_wifi(rf::Frequency center, uint16_t width_bins) const;
    void draw_bar(int y, int filled, ui::Color color);
    void update_signal_bar();
    void update_confidence_bar();
    void update_status_display();
    void update_level_display();
    void accumulate_waterfall_slice(const ChannelSpectrum& spectrum);
    void draw_waterfall_line();
    void manage_beep_audio();
    void apply_band_settings();

    std::vector<rf::Frequency> slices_{};
    std::vector<uint8_t> noise_floor_{};
    std::vector<SliceFeature> feature_{};
    size_t slice_index_{0};
    rf::Frequency band_start_{0};
    rf::Frequency band_end_{0};

    std::array<rf::Frequency, CENTER_HISTORY> center_history_{};
    size_t center_history_count_{0};
    size_t center_history_pos_{0};

    uint32_t hits_{0};
    uint16_t hold_counter_{0};
    uint16_t persistence_{0};
    uint8_t confidence_{0};
    uint8_t peak_snr_db_{0};
    uint8_t shown_snr_db_{0};
    rf::Frequency peak_freq_{0};
    uint32_t last_signal_time_{0};
    uint32_t both_band_start_time_{0};
    bool signal_updated_this_sweep_{false};
    bool wifi_like_{false};
    bool alarm_active_{false};
    uint8_t beep_divider_{0};

    bool streaming_active_{false};
    bool waterfall_enabled_{true};

    uint8_t band_mode_{1};
    uint8_t sweeping_band_{1};

    bool beep_enabled_{true};

    static std::array<ui::Color, DRONE_DETECT_WATERFALL_WIDTH> waterfall_row_;
    static std::array<uint8_t, DRONE_DETECT_WATERFALL_WIDTH> waterfall_power_;

    ui::Button button_band{
        {7 * 8, 0 * 16, 8 * 8, 16},
        "5.8GHz"};

    ui::Checkbox check_beep{
        {16 * 8, 0 * 16},
        4,
        "Beep",
        true};

    ui::Checkbox check_waterfall{
        {24 * 8, 0 * 16},
        2,
        "WF",
        true};

    ui::NumberField field_margin{
        {7 * 8, 1 * 16},
        2,
        {3, 40},
        1,
        ' '};

    ui::NumberField field_width{
        {18 * 8, 1 * 16},
        2,
        {1, 20},
        1,
        ' '};

    ui::NumberField field_alarm{
        {8 * 8, 2 * 16},
        2,
        {20, 95},
        5,
        ' '};

    ui::NumberField field_hold{
        {16 * 8, 2 * 16},
        2,
        {1, 60},
        1,
        ' '};

    ui::NumberField field_band_dwell{
        {26 * 8, 2 * 16},
        2,
        {1, 60},
        1,
        ' '};

    ui::LNAGainField field_lna_24{{6 * 8, 3 * 16}};
    ui::VGAGainField field_vga_24{{12 * 8, 3 * 16}};
    ui::LNAGainField field_lna_58{{21 * 8, 3 * 16}};
    ui::VGAGainField field_vga_58{{27 * 8, 3 * 16}};

    ui::NumberField field_start_24{
        {5 * 8, 4 * 16}, 4, {1000, 6000}, 1, ' '};
    ui::NumberField field_end_24{
        {10 * 8, 4 * 16}, 4, {1000, 6000}, 1, ' '};
    ui::NumberField field_start_58{
        {5 * 8, 5 * 16}, 4, {1000, 6000}, 1, ' '};
    ui::NumberField field_end_58{
        {10 * 8, 5 * 16}, 4, {1000, 6000}, 1, ' '};

    ui::Text text_status{
        {6 * 8, 6 * 16, 13 * 8, 16},
        "---"};

    ui::Text text_hits{
        {23 * 8, 6 * 16, 7 * 8, 16},
        "0"};

    ui::Text text_level{
        {8 * 8, 7 * 16, 22 * 8, 16},
        "Waiting for data..."};

    ui::Text text_slice{
        {0 * 8, 10 * 16, 30 * 8, 16},
        ""};

    static constexpr int bar_x = 5 * 8;
    static constexpr int bar_w = 240 - bar_x - 4;
    static constexpr int bar_h = 12;
    static constexpr int signal_bar_y = 8 * 16 + 2;
    static constexpr int conf_bar_y = 9 * 16 + 2;

    static constexpr int waterfall_y = 12 * 16;

    MessageHandlerRegistration message_handler_spectrum_config{
        Message::ID::ChannelSpectrumConfig,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const ChannelSpectrumConfigMessage*>(p);
            this->fifo_ = message.fifo;
        }};

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            if (this->streaming_active_ && this->fifo_) {
                ChannelSpectrum spectrum;
                while (fifo_->out(spectrum)) {
                    this->on_channel_spectrum(spectrum);
                }
            }
        }};

    ChannelSpectrumFIFO* fifo_{};
};

}  // namespace ui::external_app::dronedetect

#endif
