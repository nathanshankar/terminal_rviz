#include "terminal_rviz/config_helper.hpp"
#include <algorithm>
#include <map>

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

RelevantSettings ConfigHelper::get_relevant_settings(const std::string& display_name) {
    // Registry of plugins and their features
    // Mode, Color, Color2, Axis, RGB, Topic, Size, Alpha, Style, History
    static const std::map<std::string, RelevantSettings> registry = {
        {"PointCloud2",          {true,  true,  false, true,  true,  false, true,  true,  true,  false}},
        {"LaserScan",            {true,  true,  false, true,  false, false, true,  true,  true,  false}},
        {"Marker",               {true,  true,  false, false, false, true,  false, true,  false, false}},
        {"MarkerArray",          {true,  true,  false, false, false, true,  false, true,  false, false}},
        {"Map",                  {false, false, false, false, false, false, false, true,  true,  false}},
        {"Odometry",             {false, true,  false, false, false, false, true,  true,  false, true}},
        {"Path",                 {false, true,  false, false, false, false, false, true,  false, false}},
        {"Pose",                 {false, true,  false, false, false, false, true,  true,  false, false}},
        {"PoseArray",            {false, true,  false, false, false, false, true,  true,  false, false}},
        {"PoseWithCovariance",   {false, true,  false, false, false, false, true,  true,  false, false}},
        {"TF",                   {true,  true,  false, true,  false, false, true,  true,  false, false}},
        {"Temperature",          {false, true,  false, false, false, false, true,  true,  true,  false}},
        {"FluidPressure",        {false, true,  false, false, false, false, true,  true,  true,  false}},
        {"Illuminance",          {false, true,  false, false, false, false, true,  true,  true,  false}},
        {"RelativeHumidity",     {false, true,  false, false, false, false, true,  true,  true,  false}},
        {"PointStamped",         {false, true,  false, false, false, false, true,  true,  false, false}},
        {"Wrench",               {false, true,  true,  false, false, false, true,  true,  false, false}},
        {"AccelStamped",         {false, true,  false, false, false, false, true,  true,  false, false}},
        {"Effort",               {false, true,  false, false, false, false, true,  true,  false, false}},
        {"RobotModel",           {false, false, false, false, false, false, false, true,  false, false}},
        {"GridCells",            {false, true,  false, false, false, false, false, true,  false, false}},
        {"LegacyPointCloud",     {false, true,  false, false, false, false, true,  true,  false, false}}
    };

    auto it = registry.find(display_name);
    if (it != registry.end()) return it->second;
    
    RelevantSettings r;
    r.alpha = true; r.color = true;
    return r;
}

ftxui::Element ConfigHelper::render_summary(const std::string& topic, const TopicConfig& cfg) {
    std::string state = cfg.color_style;
    if (cfg.color_style == "Axis") {
        state = "Axis";
    }
    if (cfg.color_style == "Flat") state = preset_names[cfg.color_index % 10];
    if (cfg.color_style == "RGB") state = "Cloud";
    if (cfg.style == "Map") state = "Map";
    if (cfg.style == "Costmap") state = "Cost";
    
    return hbox({
        text(" " + topic) | color(Color::Green) | flex,
        text(" [" + state + "] ") | color(Color::GrayLight),
        text(" [EDIT] ") | color(Color::Yellow) | bold
    });
}

ftxui::Element ConfigHelper::render_edit_modal(const std::string& topic, const TopicConfig& cfg, 
                                             int selected_idx, int right_panel_width,
                                             const std::string& display_name) {
    auto terminal = Terminal::Size();
    Elements items;
    RelevantSettings r = get_relevant_settings(display_name);

    auto add_item = [&](int idx, Element content, bool relevant) {
        if (!relevant) return;
        if (selected_idx == idx) content = content | inverted | focus;
        items.push_back(content);
    };

    // 0: Color Mode
    if (r.mode) {
        std::string mode_text = cfg.color_style;
        if (display_name == "PointCloud2" && cfg.color_style == "RGB") mode_text = "RGB (Cloud)";
        add_item(0, hbox({ text(" Mode:    "), text(mode_text) | color(Color::Cyan) | bold }), true);
    }

    // 1: Contextual
    if (cfg.color_style == "Flat") {
        std::string label = (display_name == "Wrench") ? " Linear:  " : " Color:   ";
        add_item(1, hbox({ text(label), text(preset_names[cfg.color_index % 10]) | color(preset_colors[cfg.color_index % 10]) | bold }), r.color);
    } else if (cfg.color_style == "Axis") {
        bool show_axis_selection = r.axis && (display_name != "TF");
        add_item(1, hbox({ text(" Axis:    "), text(cfg.axis) | color(Color::Yellow) | bold }), show_axis_selection);
    }

    // 10: Color 2 (Wrench Angular)
    if (r.color_2 && cfg.color_style == "Flat") {
        add_item(10, hbox({ text(" Angular: "), text(preset_names[cfg.color_index_2 % 10]) | color(preset_colors[cfg.color_index_2 % 10]) | bold }), true);
    }

    // 2: Size (Open ended scrubber)
    add_item(2, hbox({
        text(" Size:    "),
        text("< ") | bold | color(Color::GrayDark),
        text(std::to_string(cfg.size).substr(0,5)) | color(Color::Green) | bold | size(WIDTH, EQUAL, 8),
        text(" >") | bold | color(Color::GrayDark),
        text(" (Drag)") | dim
    }), r.size);

    // 3: Alpha
    add_item(3, hbox({
        text(" Alpha:   "),
        gauge(cfg.alpha) | color(Color::Green) | size(WIDTH, EQUAL, 20),
        text(" " + std::to_string(cfg.alpha).substr(0,4)) | dim
    }), r.alpha);

    // 4: Style
    add_item(4, hbox({ text(" Style:   "), text(cfg.style) | color(Color::Orange1) | bold }), r.style);

    // 5: History
    add_item(5, hbox({ 
        text(" Hist:    "), 
        gauge(static_cast<float>(cfg.history_length) / 100.0f) | color(Color::Blue) | size(WIDTH, EQUAL, 20),
        text(" " + std::to_string(cfg.history_length)) | dim
    }), r.history);

    auto modal_content = vbox({
        text(" EDIT: " + topic) | bold | color(Color::Magenta) | hcenter,
        separator(),
        vbox(std::move(items)),
        separator(),
        hbox({
            filler(),
            text(" [ DONE ] ") | (selected_idx == 6 ? (inverted | color(Color::Green) | focus) : nothing),
            filler(),
        })
    }) | size(WIDTH, EQUAL, right_panel_width - 4) | border | bgcolor(Color::Black);

    int x = terminal.dimx - right_panel_width;
    int y = terminal.dimy - 20;

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
                                   int right_panel_width,
                                   const std::string& display_name,
                                   int mouse_dx,
                                   bool is_dragging) {
    RelevantSettings r = get_relevant_settings(display_name);
    std::vector<int> active_indices;
    
    if (r.mode) active_indices.push_back(0);
    if (cfg.color_style == "Flat") { if (r.color) active_indices.push_back(1); }
    else if (cfg.color_style == "Axis") { if (r.axis && display_name != "TF") active_indices.push_back(1); }
    if (r.color_2 && cfg.color_style == "Flat") active_indices.push_back(10);
    if (r.size) active_indices.push_back(2);
    if (r.alpha) active_indices.push_back(3);
    if (r.style) active_indices.push_back(4);
    if (r.history) active_indices.push_back(5);
    active_indices.push_back(6); // DONE

    auto it = std::find(active_indices.begin(), active_indices.end(), selected_idx);
    int current_active_idx = (it != active_indices.end()) ? std::distance(active_indices.begin(), it) : 0;
    if (it == active_indices.end()) {
        selected_idx = active_indices[0];
        current_active_idx = 0;
    }

    if (event.is_mouse()) {
        auto mouse = event.mouse();
        auto terminal = Terminal::Size();
        int start_x = terminal.dimx - right_panel_width;
        int start_y = terminal.dimy - 20;
        
        // 1. Update selection only if NOT dragging or clicking (Selection follows hover ONLY when not active)
        if (!is_dragging && mouse.button == Mouse::None) {
            if (mouse.x >= start_x && mouse.x < terminal.dimx && mouse.y >= start_y) {
                int ry = mouse.y - (start_y + 3);
                if (ry >= 0 && ry < (int)active_indices.size() - 1) {
                    selected_idx = active_indices[ry];
                } else {
                    int done_y = start_y + 3 + (int)active_indices.size() - 1 + 1;
                    if (mouse.y == done_y) {
                        selected_idx = 6;
                    }
                }
            }
        }

        // 2. Perform action on current selection if mouse button is down
        if (mouse.button == Mouse::Left) {
            // Update selection on initial Press to ensure we interact with what we clicked
            if (mouse.motion == Mouse::Pressed) {
                if (mouse.x >= start_x && mouse.x < terminal.dimx && mouse.y >= start_y) {
                    int ry = mouse.y - (start_y + 3);
                    if (ry >= 0 && ry < (int)active_indices.size() - 1) {
                        selected_idx = active_indices[ry];
                    } else {
                        int done_y = start_y + 3 + (int)active_indices.size() - 1 + 1;
                        if (mouse.y == done_y) selected_idx = 6;
                    }
                }
            }

            if (selected_idx == 2) { // Size: Scrubber
                if (is_dragging || mouse.motion == Mouse::Pressed) {
                    cfg.size = std::max(0.001f, cfg.size + mouse_dx * 0.01f);
                }
                return true;
            } else if (selected_idx == 3) { // Alpha
                if (is_dragging || mouse.motion == Mouse::Pressed) {
                    float p = std::clamp(static_cast<float>(mouse.x - (start_x + 11)) / 20.0f, 0.0f, 1.0f);
                    cfg.alpha = p;
                }
                return true;
            } else if (selected_idx == 5) { // History
                if (is_dragging || mouse.motion == Mouse::Pressed) {
                    float p = std::clamp(static_cast<float>(mouse.x - (start_x + 11)) / 20.0f, 0.0f, 1.0f);
                    cfg.history_length = static_cast<int>(p * 100.0f);
                }
                return true;
            }
            
            // For buttons/toggles, trigger on RELEASE to ensure reliable single-click interaction
            if (mouse.motion == Mouse::Released) {
                if (selected_idx == 6) show_modal = false;
                else handle_edit_event(Event::Return, cfg, selected_idx, show_modal, right_panel_width, display_name);
                return true;
            }
        }
        return (mouse.x >= start_x || is_dragging); 
    }

    if (event == Event::ArrowUp) { 
        current_active_idx = (current_active_idx - 1 + active_indices.size()) % active_indices.size();
        selected_idx = active_indices[current_active_idx];
        return true; 
    }
    if (event == Event::ArrowDown) { 
        current_active_idx = (current_active_idx + 1) % active_indices.size();
        selected_idx = active_indices[current_active_idx];
        return true; 
    }
    
    auto cycle_style = [&]() {
        if (display_name == "Map") {
            cfg.style = (cfg.style == "Map") ? "Costmap" : "Map";
        } else if (display_name == "LaserScan" || display_name == "PointCloud2" ||
                   display_name == "Temperature" || display_name == "FluidPressure" ||
                   display_name == "Illuminance" || display_name == "RelativeHumidity") {
            std::vector<std::string> styles = {"Points", "Squares", "Flat Squares", "Spheres", "Boxes", "Tiles"};
            auto it_s = std::find(styles.begin(), styles.end(), cfg.style);
            if (it_s == styles.end()) cfg.style = "Points";
            else {
                int next = (std::distance(styles.begin(), it_s) + 1) % styles.size();
                cfg.style = styles[next];
            }
        }
    };

    if (event == Event::Character('+') || event == Event::Character('=')) {
        if (selected_idx == 2) cfg.size += 0.05f;
        else if (selected_idx == 3) cfg.alpha = std::min(1.0f, cfg.alpha + 0.1f);
        else if (selected_idx == 5) cfg.history_length = std::min(100, cfg.history_length + 5);
        return true;
    }
    if (event == Event::Character('-') || event == Event::Character('_')) {
        if (selected_idx == 2) cfg.size = std::max(0.01f, cfg.size - 0.05f);
        else if (selected_idx == 3) cfg.alpha = std::max(0.1f, cfg.alpha - 0.1f);
        else if (selected_idx == 5) cfg.history_length = std::max(1, cfg.history_length - 5);
        return true;
    }

    if (event == Event::Return || event == Event::Character(' ')) {
        if (selected_idx == 6) show_modal = false;
        else {
            if (selected_idx == 0) {
                if (cfg.color_style == "Flat") {
                    if (r.axis) cfg.color_style = "Axis";
                    else if (r.topic) cfg.color_style = "Topic";
                    else cfg.color_style = "Flat";
                } else if (cfg.color_style == "Axis") {
                    if (r.rgb) cfg.color_style = "RGB";
                    else cfg.color_style = "Flat";
                } else {
                    cfg.color_style = "Flat";
                }
                handle_edit_event(Event::Custom, cfg, selected_idx, show_modal, right_panel_width, display_name);
            } else if (selected_idx == 1) {
                if (cfg.color_style == "Flat") cfg.color_index = (cfg.color_index + 1) % 10;
                else if (cfg.color_style == "Axis") {
                    if (cfg.axis == "Z") cfg.axis = "X"; else if (cfg.axis == "X") cfg.axis = "Y"; else cfg.axis = "Z";
                }
            } else if (selected_idx == 10) {
                cfg.color_index_2 = (cfg.color_index_2 + 1) % 10;
            } else if (selected_idx == 4) cycle_style();
        }
        return true;
    }

    return false;
}

} // namespace terminal_rviz
