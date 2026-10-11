/*
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "ui.hpp"
#include "ui_dronedetect.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::dronedetect {
void initialize_app(ui::NavigationView& nav) {
    nav.push<DroneDetectView>();
}
}  // namespace ui::external_app::dronedetect

extern "C" {

__attribute__((section(".external_app.app_dronedetect.application_information"), used)) application_information_t _application_information_dronedetect = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::dronedetect::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "DroneDetect",
    /*.bitmap_data = */
    {
        0x0E, 0x70,
        0x11, 0x88,
        0x11, 0x88,
        0x31, 0x8C,
        0x3E, 0x7C,
        0x40, 0x02,
        0x80, 0x01,
        0x80, 0x01,
        0x80, 0x01,
        0x40, 0x02,
        0x3E, 0x7C,
        0x31, 0x8C,
        0x11, 0x88,
        0x11, 0x88,
        0x0E, 0x70,
        0x00, 0x00},
    /*.icon_color = */ ui::Color::orange().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    // Wideband spectrum baseband image used by this application (PSPE).
    /*.m4_app_tag = portapack::spi_flash::image_tag_wideband_spectrum */ {'P', 'S', 'P', 'E'},
    /*.m4_app_offset = */ 0x00000000,  // riempito automaticamente in build
};
}
