#include "terminal_rviz/displays/rosbag_display.hpp"
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include "ftxui/dom/elements.hpp"

namespace terminal_rviz {

RosbagDisplay::RosbagDisplay(rclcpp::Node::SharedPtr node)
    : Display("Rosbag", node) {
    output_path_ = std::filesystem::current_path().string();
}

RosbagDisplay::~RosbagDisplay() {
    stop_recording();
}

void RosbagDisplay::onInitialize() {}

void RosbagDisplay::toggle_topic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (selected_topics_.count(topic)) selected_topics_.erase(topic);
    else selected_topics_.insert(topic);
}

void RosbagDisplay::set_output_path(const std::string& path) {
    std::lock_guard<std::mutex> lock(mtx_);
    output_path_ = path;
}

void RosbagDisplay::start_recording() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (recording_pid_ > 0) return;
    if (selected_topics_.empty()) { status_msg_ = "Error: No topics selected"; return; }

    recording_pid_ = fork();
    if (recording_pid_ == 0) {
        // Child process: Redirect I/O to /dev/null to avoid terminal contention
        int devnull = open("/dev/null", O_RDWR);
        if (devnull != -1) {
            dup2(devnull, STDIN_FILENO);
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        
        // Put child in its own process group
        setpgid(0, 0);

        std::vector<std::string> args = {"ros2", "bag", "record"};
        for (const auto& t : selected_topics_) args.push_back(t);
        args.push_back("-o");
        
        std::string filename = "rosbag_" + std::to_string(time(nullptr));
        std::filesystem::path p = std::filesystem::path(output_path_) / filename;
        args.push_back(p.string());

        std::vector<char*> c_args;
        for (const auto& arg : args) c_args.push_back(const_cast<char*>(arg.c_str()));
        c_args.push_back(nullptr);

        execvp("ros2", c_args.data());
        exit(1);
    } else if (recording_pid_ < 0) {
        status_msg_ = "Error: Failed to fork";
        recording_pid_ = -1;
    } else {
        status_msg_ = "Recording...";
    }
}

void RosbagDisplay::stop_recording() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (recording_pid_ <= 0) return;

    // Send SIGINT to the process group
    kill(-recording_pid_, SIGINT);
    
    int status;
    // Use WNOHANG in a short loop or just wait if we expect it to exit quickly
    for (int i = 0; i < 10; ++i) {
        if (waitpid(recording_pid_, &status, WNOHANG) > 0) break;
        usleep(100000); // 100ms
    }
    
    recording_pid_ = -1;
    status_msg_ = "Stopped. Bag saved.";
}

void RosbagDisplay::render(RvizRenderer&, ftxui::Canvas&, const std::string&, std::shared_ptr<tf2_ros::Buffer>) {}

ftxui::Element RosbagDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);

    Elements topics_ui;
    auto topic_names = node_->get_topic_names_and_types();
    std::vector<std::string> sorted_topics;
    for (auto const& [name, types] : topic_names) sorted_topics.push_back(name);
    std::sort(sorted_topics.begin(), sorted_topics.end());

    int visible_count = 0;
    int max_visible = 5;
    for (size_t i = 0; i < sorted_topics.size(); ++i) {
        if (visible_count >= config_scroll && visible_count < config_scroll + max_visible) {
            bool selected = selected_topics_.count(sorted_topics[i]);
            topics_ui.push_back(hbox({
                text(selected ? " [X] " : " [ ] ") | color(selected ? Color::Green : Color::GrayDark),
                text(sorted_topics[i])
            }));
        }
        visible_count++;
    }
    // Fill remaining space if list is short
    while (topics_ui.size() < (size_t)max_visible) topics_ui.push_back(filler());

    auto start_btn = text(" [ START ] ") | border;
    if (recording_pid_ > 0) start_btn = start_btn | dim;
    else start_btn = start_btn | bold | color(Color::Green);

    auto stop_btn = text(" [ STOP ] ") | border;
    if (recording_pid_ <= 0) stop_btn = stop_btn | dim;
    else stop_btn = stop_btn | bold | color(Color::Red) | inverted;

    return vbox({
        hbox({ text(" Rosbag Recorder ") | bold | color(Color::Yellow), filler() }),
        separator(),
        hbox({ text(" Output: ") | bold, text(output_path_) | color(Color::Cyan) | flex, text(" [CHANGE]") | color(Color::GrayLight) }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 5),
        separator(),
        hbox({
            filler(),
            start_btn,
            stop_btn,
            filler()
        }),
        text(" Status: " + status_msg_) | dim
    }) | border | size(HEIGHT, EQUAL, 14);
}

bool RosbagDisplay::handle_event(ftxui::Event event, int /*scroll_offset*/) {
    if (!event.is_mouse()) return false;
    auto mouse = event.mouse();
    if (mouse.button != ftxui::Mouse::Left || mouse.motion != ftxui::Mouse::Pressed) return false;

    // This logic needs precise hit-testing which is hard without knowing terminal size
    // For now, we'll let Visualizer handle the buttons via a more structured way if needed,
    // but we can try to estimate.
    return false; 
}

} // namespace terminal_rviz
