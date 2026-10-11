/* Passive spectrum-based drone-link detector. */

#include "ui_dronedetect.hpp"
#include "string_format.hpp"

using namespace portapack;

namespace ui::external_app::dronedetect {

std::array<ui::Color, DRONE_DETECT_WATERFALL_WIDTH> DroneDetectView::waterfall_row_{};
std::array<uint8_t, DRONE_DETECT_WATERFALL_WIDTH> DroneDetectView::waterfall_power_{};

static uint8_t spectrum_bin(const ChannelSpectrum& spectrum, size_t logical_bin) {
    return logical_bin < 120 ? spectrum.db[134 + logical_bin] : spectrum.db[logical_bin - 118];
}

// The 25th percentile limits the effect of wide signals on the noise estimate.
static uint8_t noise_percentile(const ChannelSpectrum& spectrum) {
    uint16_t histogram[256] = {0};
    for (size_t i = 0; i < DRONE_DETECT_WATERFALL_WIDTH; i++)
        histogram[spectrum_bin(spectrum, i)]++;

    const uint16_t target = DRONE_DETECT_WATERFALL_WIDTH / 4;
    uint16_t accumulated = 0;
    for (size_t value = 0; value < 256; value++) {
        accumulated += histogram[value];
        if (accumulated >= target) return static_cast<uint8_t>(value);
    }
    return 0;
}

static rf::Frequency bin_frequency(rf::Frequency slice_center, uint16_t logical_bin) {
    const rf::Frequency low = slice_center - (rf::Frequency)DRONE_DETECT_WATERFALL_WIDTH * DRONE_DETECT_BIN_HZ / 2;
    return low + (rf::Frequency)logical_bin * DRONE_DETECT_BIN_HZ;
}

static rf::Frequency freq_distance(rf::Frequency a, rf::Frequency b) {
    return a > b ? a - b : b - a;
}

static Color waterfall_color(uint8_t level) {
    if (level <= 86)
        return {0, 0, static_cast<uint8_t>(level * 255 / 86)};
    if (level <= 171) {
        const uint8_t offset = level - 86;
        return {0,
                static_cast<uint8_t>(offset * 255 / 85),
                static_cast<uint8_t>(255 - offset * 255 / 85)};
    }

    const uint8_t offset = level - 171;
    return {static_cast<uint8_t>(offset * 255 / 84),
            static_cast<uint8_t>(255 - offset * 255 / 84),
            0};
}

DroneDetectView::~DroneDetectView() {
    baseband::spectrum_streaming_stop();
    audio::output::stop();
    receiver_model.set_sampling_rate(3072000);  // Restore a stable rate for other apps.
    receiver_model.disable();
    baseband::shutdown();
}

void DroneDetectView::focus() {
    button_band.focus();
}

void DroneDetectView::paint(Painter& painter) {
    View::paint(painter);
    const Point origin = screen_pos();
    const auto& style = *Theme::getInstance()->fg_light;
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 0 * 16}, style, "Band:");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 1 * 16}, style, "Margin:");
    painter.draw_string({origin.x() + 10 * 8, origin.y() + 1 * 16}, style, "dB");
    painter.draw_string({origin.x() + 13 * 8, origin.y() + 1 * 16}, style, "W:");
    painter.draw_string({origin.x() + 21 * 8, origin.y() + 1 * 16}, style, "MHz");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 2 * 16}, style, "Alarm:");
    painter.draw_string({origin.x() + 10 * 8, origin.y() + 2 * 16}, style, "%");
    painter.draw_string({origin.x() + 11 * 8, origin.y() + 2 * 16}, style, "Hold:");
    painter.draw_string({origin.x() + 20 * 8, origin.y() + 2 * 16}, style, "Both:");
    painter.draw_string({origin.x() + 29 * 8, origin.y() + 2 * 16}, style, "s");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 3 * 16}, style, "2.4 L:");
    painter.draw_string({origin.x() + 10 * 8, origin.y() + 3 * 16}, style, "V:");
    painter.draw_string({origin.x() + 15 * 8, origin.y() + 3 * 16}, style, "5.8 L:");
    painter.draw_string({origin.x() + 25 * 8, origin.y() + 3 * 16}, style, "V:");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 4 * 16}, style, "2.4:");
    painter.draw_string({origin.x() + 9 * 8, origin.y() + 4 * 16}, style, "-");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 5 * 16}, style, "5.8:");
    painter.draw_string({origin.x() + 9 * 8, origin.y() + 5 * 16}, style, "-");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 6 * 16}, style, "State");
    painter.draw_string({origin.x() + 19 * 8, origin.y() + 6 * 16}, style, "Hit:");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 7 * 16}, style, "Level:");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 8 * 16}, style, "SNR");
    painter.draw_string({origin.x() + 0 * 8, origin.y() + 9 * 16}, style, "Conf");
    update_signal_bar();
    update_confidence_bar();
}

void DroneDetectView::manage_beep_audio() {
    if (beep_enabled_) {
        audio::set_rate(audio::Rate::Hz_24000);
        audio::output::start();
    } else {
        baseband::request_beep_stop();
        audio::output::stop();
    }
}

// Build overlapping 20 MHz slices that cover the selected band.
void DroneDetectView::build_slices() {
    slices_.clear();

    if (sweeping_band_ == 0) {
        // 2.4 GHz ISM band.
        band_start_ = (rf::Frequency)band_start_mhz_24_ * 1000000ull;
        band_end_ = (rf::Frequency)band_end_mhz_24_ * 1000000ull;
    } else {
        // 5.8 GHz band.
        band_start_ = (rf::Frequency)band_start_mhz_58_ * 1000000ull;
        band_end_ = (rf::Frequency)band_end_mhz_58_ * 1000000ull;
    }

    const rf::Frequency step = DRONE_DETECT_SLICE_BW - 2000000;  // 2 MHz overlap.
    rf::Frequency center = band_start_ + DRONE_DETECT_SLICE_BW / 2;
    while (center <= band_end_ + DRONE_DETECT_SLICE_BW / 2) {
        slices_.push_back(center);
        center += step;
    }
    if (slices_.empty())
        slices_.push_back((band_start_ + band_end_) / 2);

    noise_floor_.assign(slices_.size(), 40);
    feature_.assign(slices_.size(), SliceFeature{});
    center_history_count_ = 0;
    center_history_pos_ = 0;
    persistence_ = 0;
    waterfall_power_.fill(0);
    slice_index_ = 0;
    apply_band_settings();
}

void DroneDetectView::apply_band_settings() {
    if (sweeping_band_ == 0) {
        receiver_model.set_lna(lna_gain_24_);
        receiver_model.set_vga(vga_gain_24_);
    } else {
        receiver_model.set_lna(lna_gain_58_);
        receiver_model.set_vga(vga_gain_58_);
    }
}

void DroneDetectView::retune() {
    receiver_model.set_target_frequency(slices_[slice_index_]);
    chThdSleepMilliseconds(3);  // Allow the PLL to settle before capture.
    baseband::spectrum_streaming_start();
}

void DroneDetectView::draw_bar(int y, int filled, Color color) {
    const Point origin = screen_pos();
    const int x = origin.x() + bar_x;
    const int top = origin.y() + y;

    if (filled > 0)
        display.fill_rectangle({{x, top}, {filled, bar_h}}, color);
    if (filled < bar_w)
        display.fill_rectangle({{x + filled, top}, {bar_w - filled, bar_h}}, Color::darker_grey());
}

// Show the strongest signal's SNR on a 0-40 dB scale with slow decay.
void DroneDetectView::update_signal_bar() {
    const int filled = std::min<int>(bar_w, shown_snr_db_ * bar_w / 40);
    const Color color = shown_snr_db_ >= 30   ? Color{180, 110, 70}
                        : shown_snr_db_ >= 15 ? Color{150, 150, 70}
                                              : Color{70, 150, 120};
    draw_bar(signal_bar_y, filled, color);
}

void DroneDetectView::update_confidence_bar() {
    const int filled = std::min<int>(bar_w, confidence_ * bar_w / 100);
    const Color color = confidence_ >= 75            ? Color::red()
                        : confidence_ >= alarm_conf_ ? Color::orange()
                        : confidence_ >= 25          ? Color::yellow()
                                                     : Color::green();
    draw_bar(conf_bar_y, filled, color);
}

void DroneDetectView::update_status_display() {
    const Style* style;
    std::string label;
    const uint32_t signal_age = last_signal_time_ ? chTimeNow() - last_signal_time_ : SIGNAL_DISPLAY_HOLD_MS;

    if (signal_age >= SIGNAL_DISPLAY_HOLD_MS) {
        text_status.set("---");
        text_status.set_style(Theme::getInstance()->fg_green);
        text_hits.set(to_string_dec_uint(hits_));
        return;
    }
    if (!signal_updated_this_sweep_)
        return;

    if (confidence_ >= 75) {
        label = "DRONE!";
        style = Theme::getInstance()->fg_red;
    } else if (confidence_ >= alarm_conf_) {
        label = "LIKELY";
        style = Theme::getInstance()->fg_orange;
    } else if (confidence_ >= 25) {
        label = wifi_like_ ? "WiFi?" : "SIGNAL";
        style = Theme::getInstance()->fg_yellow;
    } else {
        label = "---";
        style = Theme::getInstance()->fg_green;
    }

    text_status.set(label + " " + to_string_dec_uint(confidence_) + "%");
    text_status.set_style(style);
    text_hits.set(to_string_dec_uint(hits_));
}

void DroneDetectView::update_level_display() {
    if (!signal_updated_this_sweep_ && last_signal_time_ &&
        chTimeNow() - last_signal_time_ < SIGNAL_DISPLAY_HOLD_MS)
        return;
    if (peak_snr_db_ == 0) {
        text_level.set("--");
        text_level.set_style(Theme::getInstance()->fg_green);
        return;
    }
    text_level.set(to_string_dec_uint(peak_snr_db_) + "dB @" + to_string_short_freq(peak_freq_));
    text_level.set_style(peak_snr_db_ >= margin_db_ ? Theme::getInstance()->fg_red : Theme::getInstance()->fg_green);
}

void DroneDetectView::accumulate_waterfall_slice(const ChannelSpectrum& spectrum) {
    if (!waterfall_enabled_) return;

    const rf::Frequency band_width = band_end_ - band_start_;
    const rf::Frequency bin_width = DRONE_DETECT_SLICE_BW / DRONE_DETECT_NB_BINS;
    const rf::Frequency usable_width = DRONE_DETECT_WATERFALL_WIDTH * bin_width;
    const rf::Frequency slice_low = slices_[slice_index_] - usable_width / 2;
    const rf::Frequency owned_low = slice_index_ == 0
                                        ? band_start_
                                        : (slices_[slice_index_ - 1] + slices_[slice_index_]) / 2;
    const rf::Frequency owned_high = slice_index_ + 1 == slices_.size()
                                         ? band_end_
                                         : (slices_[slice_index_] + slices_[slice_index_ + 1]) / 2;

    for (size_t bin = 0; bin < DRONE_DETECT_WATERFALL_WIDTH; bin++) {
        const rf::Frequency frequency = slice_low + bin * bin_width;
        if (frequency < owned_low || frequency >= owned_high) continue;

        const size_t x = (frequency - band_start_) * DRONE_DETECT_WATERFALL_WIDTH / band_width;
        const uint8_t power = spectrum_bin(spectrum, bin);
        if (x < waterfall_power_.size()) waterfall_power_[x] = power;
    }
}

void DroneDetectView::draw_waterfall_line() {
    if (!waterfall_enabled_) return;

    for (size_t x = 0; x < waterfall_row_.size(); x++)
        waterfall_row_[x] = waterfall_color(waterfall_power_[x]);
    display.draw_pixels({{0, display.scroll(1)}, {(int)waterfall_row_.size(), 1}}, waterfall_row_);
    waterfall_power_.fill(0);
}

void DroneDetectView::analyze_slice(const ChannelSpectrum& spectrum) {
    auto& feature = feature_[slice_index_];

    const uint8_t floor_now = noise_percentile(spectrum);
    noise_floor_[slice_index_] = static_cast<uint8_t>((noise_floor_[slice_index_] * 7 + floor_now) / 8);
    feature.noise = noise_floor_[slice_index_];

    const uint16_t threshold = std::min<uint16_t>(255, feature.noise + margin_db_ * DRONE_DETECT_COUNTS_PER_DB);
    const uint16_t min_bins = std::max<uint16_t>(1, (min_width_mhz_ * 1000000u) / DRONE_DETECT_BIN_HZ);

    // Measure the longest contiguous above-threshold run as the occupied channel.
    uint16_t run = 0, best_run = 0, best_end = 0;
    uint8_t peak = 0;
    for (size_t i = 0; i < DRONE_DETECT_WATERFALL_WIDTH; i++) {
        const uint8_t value = spectrum_bin(spectrum, i);
        if (value > peak) peak = value;
        if (value > threshold) {
            run++;
            if (run > best_run) {
                best_run = run;
                best_end = i;
            }
        } else {
            run = 0;
        }
    }

    feature.peak = peak;
    feature.width = best_run;
    feature.center_bin = best_run ? static_cast<uint16_t>(best_end - best_run / 2) : 0;
    feature.active = (best_run >= min_bins) && (peak > threshold);
}

void DroneDetectView::on_channel_spectrum(const ChannelSpectrum& spectrum) {
    baseband::spectrum_streaming_stop();

    analyze_slice(spectrum);
    accumulate_waterfall_slice(spectrum);

    text_slice.set(to_string_short_freq(slices_[slice_index_]) + " (" +
                   (sweeping_band_ == 0 ? "2.4G" : "5.8G") + ")");

    slice_index_++;
    if (slice_index_ >= slices_.size()) {
        end_of_sweep();
    } else {
        retune();
    }
}

// A wide, fixed signal on a standard 802.11 channel is penalized, not excluded.
bool DroneDetectView::looks_like_wifi(rf::Frequency center, uint16_t width_bins) const {
    if ((uint32_t)width_bins * DRONE_DETECT_BIN_HZ < 15000000u) return false;

    const rf::Frequency first = sweeping_band_ == 0 ? 2412000000LL : 5745000000LL;
    const rf::Frequency last = sweeping_band_ == 0 ? 2472000000LL : 5825000000LL;
    const rf::Frequency step = sweeping_band_ == 0 ? 5000000LL : 20000000LL;

    rf::Frequency nearest = last;
    for (rf::Frequency channel = first; channel <= last; channel += step)
        nearest = std::min(nearest, freq_distance(center, channel));

    return nearest <= 2500000LL;
}

void DroneDetectView::evaluate_confidence() {
    const uint16_t wide_bins = 6000000u / DRONE_DETECT_BIN_HZ;    // >=6 MHz: video-like activity.
    const uint16_t narrow_bins = 3000000u / DRONE_DETECT_BIN_HZ;  // <=3 MHz: control-like activity.

    size_t active_count = 0, best_slice = 0;
    bool wide_present = false, narrow_present = false;
    int best_snr = 0;

    for (size_t i = 0; i < feature_.size(); i++) {
        const auto& feature = feature_[i];
        if (!feature.active) continue;

        active_count++;
        if (feature.width >= wide_bins)
            wide_present = true;
        else if (feature.width <= narrow_bins)
            narrow_present = true;

        const int snr = (int)feature.peak - (int)feature.noise;
        if (snr > best_snr) {
            best_snr = snr;
            best_slice = i;
        }
    }

    peak_snr_db_ = static_cast<uint8_t>(std::min(99, best_snr / DRONE_DETECT_COUNTS_PER_DB));

    if (peak_snr_db_ > shown_snr_db_)
        shown_snr_db_ = peak_snr_db_;
    else if (shown_snr_db_ > 0)
        shown_snr_db_--;

    signal_updated_this_sweep_ = active_count != 0;
    if (active_count) {
        last_signal_time_ = chTimeNow();
        peak_freq_ = bin_frequency(slices_[best_slice], feature_[best_slice].center_bin);
        center_history_[center_history_pos_] = peak_freq_;
        center_history_pos_ = (center_history_pos_ + 1) % CENTER_HISTORY;
        if (center_history_count_ < CENTER_HISTORY) center_history_count_++;
        if (persistence_ < 100) persistence_++;
    } else {
        peak_freq_ = 0;
        center_history_count_ = 0;
        center_history_pos_ = 0;
        if (persistence_) persistence_--;
    }

    // Count changes greater than 4 MHz in the strongest signal's center frequency.
    size_t hops = 0;
    for (size_t k = 1; k < center_history_count_; k++) {
        const size_t a = (center_history_pos_ + CENTER_HISTORY - center_history_count_ + k - 1) % CENTER_HISTORY;
        const size_t b = (a + 1) % CENTER_HISTORY;
        if (freq_distance(center_history_[a], center_history_[b]) > 4000000LL) hops++;
    }

    wifi_like_ = active_count && (hops == 0) &&
                 looks_like_wifi(peak_freq_, feature_[best_slice].width);

    int score = 0;
    if (active_count) {
        score = 20;
        if (hops >= 3)
            score += 30;
        else if (hops == 2)
            score += 22;
        else if (hops == 1)
            score += 12;

        // Wide and narrow activity in one sweep supports a drone-link hypothesis.
        if (wide_present && narrow_present)
            score += 20;
        else if (narrow_present)
            score += 8;

        if (active_count >= 2) score += 10;
        if (persistence_ >= 4) score += 10;
        if (peak_snr_db_ >= 20) score += 5;
        if (wifi_like_) score = std::min(score - 35, 45);
    }
    score = std::max(0, std::min(100, score));

    // Rise immediately and decay gradually to keep the indicator readable.
    confidence_ = (score > confidence_) ? static_cast<uint8_t>(score)
                                        : static_cast<uint8_t>((confidence_ * 3 + score) / 4);

    if (confidence_ >= alarm_conf_) {
        if (!alarm_active_) hits_++;
        alarm_active_ = true;
        hold_counter_ = hold_frames_;
    } else if (hold_counter_ > 0) {
        hold_counter_--;
    } else {
        alarm_active_ = false;
    }
}

void DroneDetectView::handle_alarm() {
    if (!beep_enabled_ || !alarm_active_) {
        beep_divider_ = 0;
        return;
    }

    if (beep_divider_) {
        beep_divider_--;
        return;
    }

    // High confidence uses a fast, high-pitched alarm; medium confidence beeps slowly.
    const bool urgent = confidence_ >= 85;
    beep_divider_ = urgent ? 0 : (confidence_ >= 70 ? 1 : 4);
    baseband::request_audio_beep(urgent ? 2600 : 1500, 24000, urgent ? 150 : 80);
}

void DroneDetectView::end_of_sweep() {
    draw_waterfall_line();
    evaluate_confidence();
    handle_alarm();

    update_status_display();
    update_level_display();
    update_signal_bar();
    update_confidence_bar();

    if (band_mode_ == 2) {
        const uint32_t dwell_ms = static_cast<uint32_t>(band_dwell_seconds_) * 1000;
        if (chTimeNow() - both_band_start_time_ >= dwell_ms) {
            // In Both mode, remain on each band for the selected dwell time.
            sweeping_band_ = sweeping_band_ == 0 ? 1 : 0;
            both_band_start_time_ = chTimeNow();
            build_slices();
        } else {
            slice_index_ = 0;
        }
    } else {
        slice_index_ = 0;
    }
    retune();
}

void DroneDetectView::on_show() {
    streaming_active_ = true;
    if (waterfall_enabled_)
        display.scroll_set_area(waterfall_y, screen_height - 1);
    else
        display.scroll_disable();
    baseband::spectrum_streaming_start();
}

void DroneDetectView::on_hide() {
    streaming_active_ = false;
    baseband::spectrum_streaming_stop();
    display.scroll_disable();
}

DroneDetectView::DroneDetectView(NavigationView& nav) {
    (void)nav;
    baseband::run_image(portapack::spi_flash::image_tag_wideband_spectrum);

    add_children({&button_band,
                  &check_beep,
                  &field_margin,
                  &field_width,
                  &field_alarm,
                  &field_hold,
                  &field_band_dwell,
                  &field_lna_24,
                  &field_vga_24,
                  &field_lna_58,
                  &field_vga_58,
                  &field_start_24,
                  &field_end_24,
                  &field_start_58,
                  &field_end_58,
                  &text_status,
                  &text_hits,
                  &text_level,
                  &text_slice,
                  &check_waterfall});

    baseband::set_spectrum(DRONE_DETECT_SLICE_BW, 31);
    receiver_model.set_sampling_rate(DRONE_DETECT_SLICE_BW);
    receiver_model.set_baseband_bandwidth(DRONE_DETECT_SLICE_BW);
    receiver_model.set_squelch_level(0);
    receiver_model.enable();

    sweeping_band_ = 0;
    both_band_start_time_ = chTimeNow();
    build_slices();

    button_band.on_select = [this](Button&) {
        band_mode_ = (band_mode_ + 1) % 3;
        sweeping_band_ = (band_mode_ == 1) ? 1 : 0;
        button_band.set_text(band_mode_ == 0 ? "2.4GHz" : band_mode_ == 1 ? "5.8GHz" : "Both");
        hits_ = 0;
        confidence_ = 0;
        alarm_active_ = false;
        hold_counter_ = 0;
        shown_snr_db_ = 0;
        peak_snr_db_ = 0;
        last_signal_time_ = 0;
        signal_updated_this_sweep_ = false;
        both_band_start_time_ = chTimeNow();
        build_slices();
        update_status_display();
        update_level_display();
        update_signal_bar();
        update_confidence_bar();
        baseband::spectrum_streaming_stop();
        retune();
    };

    check_beep.set_value(beep_enabled_);
    check_beep.on_select = [this](Checkbox&, bool enabled) {
        beep_enabled_ = enabled;
        manage_beep_audio();
    };

    field_margin.set_value(margin_db_, false);
    field_margin.on_change = [this](int32_t v) { margin_db_ = static_cast<uint8_t>(v); };

    field_width.set_value(min_width_mhz_, false);
    field_width.on_change = [this](int32_t v) { min_width_mhz_ = static_cast<uint8_t>(v); };

    field_alarm.set_value(alarm_conf_, false);
    field_alarm.on_change = [this](int32_t v) { alarm_conf_ = static_cast<uint8_t>(v); };

    field_hold.set_value(hold_frames_, false);
    field_hold.on_change = [this](int32_t v) { hold_frames_ = static_cast<uint16_t>(v); };

    field_band_dwell.set_value(band_dwell_seconds_, false);
    field_band_dwell.on_change = [this](int32_t v) {
        band_dwell_seconds_ = static_cast<uint8_t>(v);
        both_band_start_time_ = chTimeNow();
    };

    field_lna_24.set_value(lna_gain_24_);
    field_lna_24.on_change = [this](int32_t v) {
        lna_gain_24_ = static_cast<uint8_t>(v);
        if (sweeping_band_ == 0) receiver_model.set_lna(lna_gain_24_);
    };
    field_vga_24.set_value(vga_gain_24_);
    field_vga_24.on_change = [this](int32_t v) {
        vga_gain_24_ = static_cast<uint8_t>(v);
        if (sweeping_band_ == 0) receiver_model.set_vga(vga_gain_24_);
    };
    field_lna_58.set_value(lna_gain_58_);
    field_lna_58.on_change = [this](int32_t v) {
        lna_gain_58_ = static_cast<uint8_t>(v);
        if (sweeping_band_ == 1) receiver_model.set_lna(lna_gain_58_);
    };
    field_vga_58.set_value(vga_gain_58_);
    field_vga_58.on_change = [this](int32_t v) {
        vga_gain_58_ = static_cast<uint8_t>(v);
        if (sweeping_band_ == 1) receiver_model.set_vga(vga_gain_58_);
    };

    field_start_24.set_value(band_start_mhz_24_, false);
    field_start_24.on_change = [this](int32_t v) {
        band_start_mhz_24_ = static_cast<uint16_t>(v);
        if (band_start_mhz_24_ >= band_end_mhz_24_) {
            band_start_mhz_24_ = band_end_mhz_24_ - 1;
            field_start_24.set_value(band_start_mhz_24_, false);
        }
        if (sweeping_band_ == 0) build_slices();
    };
    field_end_24.set_value(band_end_mhz_24_, false);
    field_end_24.on_change = [this](int32_t v) {
        band_end_mhz_24_ = static_cast<uint16_t>(v);
        if (band_end_mhz_24_ <= band_start_mhz_24_) {
            band_end_mhz_24_ = band_start_mhz_24_ + 1;
            field_end_24.set_value(band_end_mhz_24_, false);
        }
        if (sweeping_band_ == 0) build_slices();
    };
    field_start_58.set_value(band_start_mhz_58_, false);
    field_start_58.on_change = [this](int32_t v) {
        band_start_mhz_58_ = static_cast<uint16_t>(v);
        if (band_start_mhz_58_ >= band_end_mhz_58_) {
            band_start_mhz_58_ = band_end_mhz_58_ - 1;
            field_start_58.set_value(band_start_mhz_58_, false);
        }
        if (sweeping_band_ == 1) build_slices();
    };
    field_end_58.set_value(band_end_mhz_58_, false);
    field_end_58.on_change = [this](int32_t v) {
        band_end_mhz_58_ = static_cast<uint16_t>(v);
        if (band_end_mhz_58_ <= band_start_mhz_58_) {
            band_end_mhz_58_ = band_start_mhz_58_ + 1;
            field_end_58.set_value(band_end_mhz_58_, false);
        }
        if (sweeping_band_ == 1) build_slices();
    };

    check_waterfall.set_value(waterfall_enabled_);
    check_waterfall.on_select = [this](Checkbox&, bool enabled) {
        waterfall_enabled_ = enabled;
        if (enabled && streaming_active_)
            display.scroll_set_area(waterfall_y, screen_height - 1);
        else
            display.scroll_disable();
    };

    manage_beep_audio();

    retune();
}

}  // namespace ui::external_app::dronedetect
