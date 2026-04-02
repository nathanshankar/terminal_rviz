#ifndef TERMINAL_RVIZ_CONFIG_HELPER_HPP_
#define TERMINAL_RVIZ_CONFIG_HELPER_HPP_

#include <string>
#include <vector>
#include "ftxui/dom/elements.hpp"
#include "ftxui/component/event.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

struct RelevantSettings {
    bool mode = false;   // Shows the "Mode" toggle
    bool color = true;  // Shows "Color: [Preset]" when in Flat mode
    bool color_2 = false; // Shows "Color 2: [Preset]" (e.g. Wrench Angular)
    bool axis = false;   // Allows "Axis" mode
    bool rgb = false;    // Allows "RGB (Cloud)" mode
    bool topic = false;  // Allows "Topic" mode (e.g. Markers)
    
    bool size = false;
    bool alpha = true; 
    bool style = false;
    bool history = false;
};

class ConfigHelper {
public:
    static const std::vector<ftxui::Color> preset_colors;
    static const std::vector<std::string> preset_names;

    static RelevantSettings get_relevant_settings(const std::string& display_name);

    static ftxui::Element render_summary(const std::string& topic, const TopicConfig& cfg);
    
    static ftxui::Element render_edit_modal(const std::string& topic, const TopicConfig& cfg, 
                                          int selected_idx, int right_panel_width,
                                          const std::string& display_name);

    // Returns true if handled, updates cfg
    static bool handle_edit_event(ftxui::Event event, TopicConfig& cfg, 
                                int& selected_idx, bool& show_modal,
                                int right_panel_width,
                                const std::string& display_name);
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_CONFIG_HELPER_HPP_
