#ifndef TERMINAL_RVIZ_CONFIG_HELPER_HPP_
#define TERMINAL_RVIZ_CONFIG_HELPER_HPP_

#include <string>
#include <vector>
#include "ftxui/dom/elements.hpp"
#include "ftxui/component/event.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class ConfigHelper {
public:
    static ftxui::Element render_summary(const std::string& topic, const TopicConfig& cfg);
    
    static ftxui::Element render_edit_modal(const std::string& topic, const TopicConfig& cfg, 
                                          int selected_idx, int right_panel_width);

    // Returns true if handled, updates cfg
    static bool handle_edit_event(ftxui::Event event, TopicConfig& cfg, 
                                int& selected_idx, bool& show_modal,
                                int right_panel_width);
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_CONFIG_HELPER_HPP_
