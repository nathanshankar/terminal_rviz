#include "terminal_rviz/config_helper.hpp"
#include <algorithm>

namespace terminal_rviz {

using namespace ftxui;

const std::vector<Color> ConfigHelper::preset_colors = {
    Color::White, Color::Red, Color::Green, Color::Blue, Color::Yellow,
    Color::Cyan, Color::Magenta, Color::Orange1, Color::Green1, Color::DeepPink1
};

const std::vector<std::string> ConfigHelper::preset_names = {
    "White", "Red", "Green", "Blue", "Yellow",
    "Cyan", "Magenta", "Orange", "Lime", "Pink"
};

ftxui::Element ConfigHelper::render_summary(const std::string& topic, const TopicConfig& cfg) {
    std::string state = cfg.color_style;
    if (cfg.color_style == "Axis") state += "-" + cfg.axis;
    if (cfg.color_style == "Flat") state = preset_names[cfg.color_index % 10];
    
    return hbox({
        text(" " + topic) | color(Color::Green) | flex,
        text(" [" + state + "] ") | color(Color::GrayLight),
        text(" [EDIT] ") | color(Color::Yellow) | bold
    });
}

ftxui::Element ConfigHelper::render_edit_modal(const std::string& topic, const TopicConfig& cfg, 
                                             int selected_idx, int right_panel_width) {
    auto terminal = Terminal::Size();
    
    Elements items;
    // 0: Style (Always show)
    auto t_style = hbox({ text(" Style: "), text(cfg.color_style) | color(Color::Cyan) | bold });
    if (selected_idx == 0) t_style = t_style | inverted | focus;
    items.push_back(t_style);

    // 1: Contextual Setting (Axis OR Color)
    if (cfg.color_style == "Axis") {
        auto t_axis = hbox({ text(" Axis:  "), text(cfg.axis) | color(Color::Yellow) | bold });
        if (selected_idx == 1) t_axis = t_axis | inverted | focus;
        items.push_back(t_axis);
    } else if (cfg.color_style == "Flat") {
        auto t_color = hbox({ text(" Color: "), text(preset_names[cfg.color_index % 10]) | color(preset_colors[cfg.color_index % 10]) | bold });
        if (selected_idx == 1) t_color = t_color | inverted | focus;
        items.push_back(t_color);
    } else {
        items.push_back(text(" (No Mode Settings)") | dim);
    }

    // 2: Size Slider
    auto t_size = hbox({
        text(" Size:  "),
        gauge(cfg.size / 0.2f) | color(Color::Green) | size(WIDTH, EQUAL, 20),
        text(" " + std::to_string(cfg.size).substr(0,4)) | dim
    });
    if (selected_idx == 2) t_size = t_size | inverted | focus;
    items.push_back(t_size);

    // 3: Alpha Slider
    auto t_alpha = hbox({
        text(" Alpha: "),
        gauge(cfg.alpha) | color(Color::Green) | size(WIDTH, EQUAL, 20),
        text(" " + std::to_string(cfg.alpha).substr(0,4)) | dim
    });
    if (selected_idx == 3) t_alpha = t_alpha | inverted | focus;
    items.push_back(t_alpha);

    auto modal_content = vbox({
        text(" EDIT: " + topic) | bold | color(Color::Magenta) | hcenter,
        separator(),
        vbox(std::move(items)),
        separator(),
        hbox({
            filler(),
            text(" [ DONE ] ") | (selected_idx == 4 ? (inverted | color(Color::Green)) : nothing),
            filler(),
        })
    }) | size(WIDTH, EQUAL, right_panel_width - 4) | border | bgcolor(Color::Black);

    int x = terminal.dimx - right_panel_width;
    int y = terminal.dimy - 14;

    return vbox({
        filler() | size(HEIGHT, EQUAL, y),
        hbox({
            filler() | size(WIDTH, EQUAL, x),
            modal_content,
            filler(),
        }),
        filler(),
    });
}

bool ConfigHelper::handle_edit_event(ftxui::Event event, TopicConfig& cfg, 
                                   int& selected_idx, bool& show_modal,
                                   int right_panel_width) {
    if (event == Event::ArrowUp) { selected_idx = std::max(0, selected_idx - 1); return true; }
    if (event == Event::ArrowDown) { selected_idx = std::min(4, selected_idx + 1); return true; }
    
    if (event == Event::Character('+') || event == Event::Character('=')) {
        if (selected_idx == 2) cfg.size += 0.01f;
        else if (selected_idx == 3) cfg.alpha = std::min(1.0f, cfg.alpha + 0.1f);
        return true;
    }
    if (event == Event::Character('-') || event == Event::Character('_')) {
        if (selected_idx == 2) cfg.size = std::max(0.01f, cfg.size - 0.01f);
        else if (selected_idx == 3) cfg.alpha = std::max(0.1f, cfg.alpha - 0.1f);
        return true;
    }

    if (event == Event::Return || event == Event::Character(' ')) {
        if (selected_idx == 4) show_modal = false;
        else {
            if (selected_idx == 0) { // Style Cycle
                if (cfg.color_style == "Flat") cfg.color_style = "Axis";
                else if (cfg.color_style == "Axis") cfg.color_style = "RGB";
                else cfg.color_style = "Flat";
            } else if (selected_idx == 1) { // Contextual Cycle
                if (cfg.color_style == "Axis") {
                    if (cfg.axis == "Z") cfg.axis = "X";
                    else if (cfg.axis == "X") cfg.axis = "Y";
                    else cfg.axis = "Z";
                } else if (cfg.color_style == "Flat") {
                    cfg.color_index = (cfg.color_index + 1) % 10;
                }
            } else if (selected_idx == 2) { // Size
                cfg.size += 0.01f; if (cfg.size > 0.2f) cfg.size = 0.01f;
            } else if (selected_idx == 3) { // Alpha
                cfg.alpha -= 0.1f; if (cfg.alpha < 0.1f) cfg.alpha = 1.0f;
            }
        }
        return true;
    }

    if (event.is_mouse()) {
        auto mouse = event.mouse();
        auto terminal = Terminal::Size();
        int start_x = terminal.dimx - right_panel_width;
        int start_y = terminal.dimy - 14;
        
        if (mouse.x >= start_x && mouse.x < terminal.dimx && mouse.y >= start_y && mouse.y < terminal.dimy) {
            int ry = mouse.y - (start_y + 3);
            if (ry >= 0 && ry < 4) {
                selected_idx = ry;
                if (mouse.button == Mouse::Left) {
                    if (ry == 2 || ry == 3) { // Sliders
                        float percent = std::clamp(static_cast<float>(mouse.x - (start_x + 9)) / 20.0f, 0.0f, 1.0f);
                        if (ry == 2) cfg.size = percent * 0.2f;
                        else cfg.alpha = std::max(0.1f, percent);
                    } else if (mouse.motion == Mouse::Pressed) {
                        handle_edit_event(Event::Return, cfg, selected_idx, show_modal, right_panel_width);
                    }
                }
            } else if (mouse.y == start_y + 8) {
                selected_idx = 4;
                if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) show_modal = false;
            }
            return true;
        }
    }
    return false;
}

} // namespace terminal_rviz
