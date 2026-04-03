#include "terminal_rviz/displays/rosbag_display.hpp"
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <cctype>
#include "ftxui/dom/elements.hpp"

namespace terminal_rviz {

RosbagDisplay::RosbagDisplay(rclcpp::Node::SharedPtr node)
    : Display("Rosbag", node) {
    output_path_ = std::filesystem::current_path().string();
    input_path_ = std::filesystem::current_path().string();
    last_seek_time_ = std::chrono::steady_clock::now();
    last_service_call_time_ = std::chrono::steady_clock::now();

    seek_client_ = node_->create_client<rosbag2_interfaces::srv::Seek>("/rosbag2_player/seek");
    toggle_client_ = node_->create_client<rosbag2_interfaces::srv::TogglePaused>("/rosbag2_player/toggle_paused");
    rate_client_ = node_->create_client<rosbag2_interfaces::srv::SetRate>("/rosbag2_player/set_rate");
    pause_client_ = node_->create_client<rosbag2_interfaces::srv::Pause>("/rosbag2_player/pause");
    resume_client_ = node_->create_client<rosbag2_interfaces::srv::Resume>("/rosbag2_player/resume");
}

RosbagDisplay::~RosbagDisplay() {
    stop_recording();
    stop_playback();
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

static long long extract_ns(const std::string& line) {
    std::string val = "";
    bool found_digit = false;
    for (char c : line) {
        if (std::isdigit(c)) { val += c; found_digit = true; }
        else if (found_digit) break;
    }
    return val.empty() ? 0 : std::stoll(val);
}

void RosbagDisplay::set_input_path(const std::string& path) {
    std::lock_guard<std::mutex> lock(mtx_);
    input_path_ = path;
    bag_duration_ns_ = 0;
    bag_start_time_ns_ = 0;
    current_progress_ = 0.0f;
    
    try {
        std::filesystem::path meta = std::filesystem::path(input_path_) / "metadata.yaml";
        if (std::filesystem::exists(meta)) {
            std::ifstream f(meta);
            std::string line;
            int state = 0; // 1: duration, 2: starting_time
            while (std::getline(f, line)) {
                int line_state = 0;
                if (line.find("duration:") != std::string::npos) line_state = 1;
                else if (line.find("starting_time:") != std::string::npos) line_state = 2;
                
                int active_state = (line_state > 0) ? line_state : state;
                if (active_state > 0) {
                    if (line.find("nanoseconds") != std::string::npos) {
                        long long val = extract_ns(line.substr(line.find("nanoseconds")));
                        if (val > 0) {
                            if (active_state == 1) bag_duration_ns_ = val;
                            else bag_start_time_ns_ = val;
                            state = 0; 
                        }
                    } else {
                        state = active_state;
                    }
                }
            }
        }
    } catch (...) {}

    if (bag_duration_ns_ > 0) {
        status_msg_ = "Loaded Bag (" + std::to_string(bag_duration_ns_ / 1000000000LL) + "s)";
    } else {
        status_msg_ = "Bag selected (Waiting for Metadata)";
    }
}

void RosbagDisplay::toggle_pause() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (playback_pid_ <= 0) return;
    
    is_paused_ = !is_paused_;
    if (toggle_client_->service_is_ready()) {
        auto req = std::make_shared<rosbag2_interfaces::srv::TogglePaused::Request>();
        toggle_client_->async_send_request(req);
    } else {
        // Fallback to command line if client not ready
        std::string cmd = "ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused \"{}\" > /dev/null 2>&1 &";
        (void)!system(cmd.c_str());
    }
    status_msg_ = is_paused_ ? "Paused" : "Resumed";
}

void RosbagDisplay::start_scrubbing(float progress) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (playback_pid_ <= 0) {
        is_seeking_ = true;
        current_progress_ = progress;
        was_playing_before_scrub_ = false;
        return;
    }
    was_playing_before_scrub_ = !is_paused_;
    is_seeking_ = true;
    
    if (!is_paused_) {
        if (pause_client_->service_is_ready()) {
            auto req = std::make_shared<rosbag2_interfaces::srv::Pause::Request>();
            pause_client_->async_send_request(req);
        } else {
            std::string cmd = "ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause \"{}\" > /dev/null 2>&1 &";
            (void)!system(cmd.c_str());
        }
        is_paused_ = true;
    }
}

void RosbagDisplay::finish_scrubbing() {
    bool inactive = false;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (playback_pid_ <= 0) inactive = true;
        is_seeking_ = false;
    }

    if (inactive) {
        // If it was finished or stopped, seek(force=true) will restart it
        seek(current_progress_, true);
        return;
    }
    
    // Perform the final seek at the released position - FORCE it to ensure it completes
    seek(current_progress_, true);
    
    std::lock_guard<std::mutex> lock(mtx_);
    last_seek_time_ = std::chrono::steady_clock::now();

    if (was_playing_before_scrub_) {
        if (resume_client_->service_is_ready()) {
            auto req = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();
            resume_client_->async_send_request(req);
        } else {
            std::string cmd = "ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume \"{}\" > /dev/null 2>&1 &";
            (void)!system(cmd.c_str());
        }
        is_paused_ = false;
        status_msg_ = "Playing";
    }
}

void RosbagDisplay::toggle_loop() {
    std::unique_lock<std::mutex> lock(mtx_);
    is_looping_ = !is_looping_;
    if (playback_pid_ > 0) {
        lock.unlock();
        stop_playback();
        start_playback();
    }
}

void RosbagDisplay::set_playback_rate(float rate) {
    std::lock_guard<std::mutex> lock(mtx_);
    playback_rate_ = std::clamp(rate, 0.1f, 10.0f);
    if (playback_pid_ > 0) {
        if (rate_client_->service_is_ready()) {
            auto req = std::make_shared<rosbag2_interfaces::srv::SetRate::Request>();
            req->rate = playback_rate_;
            rate_client_->async_send_request(req);
        } else {
            std::string cmd = "ros2 service call /rosbag2_player/set_rate rosbag2_interfaces/srv/SetRate \"{rate: " + std::to_string(playback_rate_) + "}\" > /dev/null 2>&1 &";
            (void)!system(cmd.c_str());
        }
    }
}

void RosbagDisplay::seek(float progress, bool force) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (bag_duration_ns_ <= 0) return;
    
    // If we are finished or stopped, and the user seeks (especially forcing it),
    // we should restart the bag at that position.
    if (playback_pid_ <= 0) {
        if (force) {
            // Drop lock before calling start_playback to avoid deadlock
            mtx_.unlock();
            start_playback(progress);
            mtx_.lock();
        } else {
            current_progress_ = progress;
        }
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    
    // Throttle seek calls during scrubbing (max 10 Hz)
    if (!force) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_service_call_time_).count();
        if (elapsed < 100) return;
    }
    
    current_progress_ = progress;
    last_seek_target_progress_ = progress;
    last_seek_time_ = now;
    last_service_call_time_ = now;
    
    long long target_ns = bag_start_time_ns_ + (long long)(progress * bag_duration_ns_);
    long long sec = target_ns / 1000000000LL;
    long long nsec = target_ns % 1000000000LL;

    if (seek_client_->service_is_ready()) {
        auto req = std::make_shared<rosbag2_interfaces::srv::Seek::Request>();
        req->time.sec = static_cast<int32_t>(sec);
        req->time.nanosec = static_cast<uint32_t>(nsec);
        seek_client_->async_send_request(req);
    } else {
        std::string cmd = "ros2 service call /rosbag2_player/seek rosbag2_interfaces/srv/Seek \"{time: {sec: " + 
                        std::to_string(sec) + ", nanosec: " + std::to_string(nsec) + "}}\" > /dev/null 2>&1";
        if (!force) cmd += " &";
        (void)!system(cmd.c_str());
    }
}

void RosbagDisplay::clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (bag_duration_ns_ <= 0 || is_seeking_) return;
    
    long long current_ns = msg->clock.sec * 1000000000LL + msg->clock.nanosec;
    
    // Ignore updates for 1 second after scrubbing to let player stabilize
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_seek_time_).count();
    if (elapsed_ms < 1000) {
        // During the stabilization period, only accept the clock if it's "close" to our seek target
        if (last_seek_target_progress_ >= 0) {
            long long target_ns = bag_start_time_ns_ + (long long)(last_seek_target_progress_ * bag_duration_ns_);
            if (std::abs(current_ns - target_ns) > 2000000000LL) { // More than 2 seconds away
                return; 
            }
        } else {
            return;
        }
    }

    if (bag_start_time_ns_ == 0) {
        // Only initialize start time if we are not in the middle of a seek and the clock is likely the start
        bag_start_time_ns_ = current_ns;
    }

    float p = (float)(current_ns - bag_start_time_ns_) / bag_duration_ns_;
    current_progress_ = std::clamp(p, 0.0f, 1.0f);
}

void RosbagDisplay::start_recording() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (recording_pid_ > 0) return;
    if (selected_topics_.empty()) { status_msg_ = "Error: No topics selected"; return; }

    recording_pid_ = fork();
    if (recording_pid_ == 0) {
        int devnull = open("/dev/null", O_RDWR);
        if (devnull != -1) {
            dup2(devnull, STDIN_FILENO);
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
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
        _exit(1);
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
    kill(-recording_pid_, SIGINT);
    int status;
    for (int i = 0; i < 10; ++i) {
        if (waitpid(recording_pid_, &status, WNOHANG) > 0) break;
        usleep(100000);
    }
    recording_pid_ = -1;
    status_msg_ = "Stopped. Bag saved.";
}

void RosbagDisplay::start_playback(float start_progress, bool start_paused) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (playback_pid_ > 0) return;
    if (!std::filesystem::exists(input_path_)) { status_msg_ = "Error: Bag path not found"; return; }

    playback_pid_ = fork();
    if (playback_pid_ == 0) {
        int devnull = open("/dev/null", O_RDWR);
        if (devnull != -1) {
            dup2(devnull, STDIN_FILENO);
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        setpgid(0, 0);
        
        std::string r_str = std::to_string(playback_rate_);
        std::vector<std::string> args = {"ros2", "bag", "play", input_path_, "--clock", "-r", r_str};
        if (is_looping_) args.push_back("--loop");
        if (start_paused) args.push_back("--start-paused");
        
        if (start_progress > 0.0f && start_progress <= 1.0f && bag_duration_ns_ > 0) {
            float offset_s = start_progress * (bag_duration_ns_ / 1000000000.0f);
            args.push_back("--start-offset");
            args.push_back(std::to_string(offset_s));
        }
        
        std::vector<char*> c_args;
        for (const auto& arg : args) c_args.push_back(const_cast<char*>(arg.c_str()));
        c_args.push_back(nullptr);
        execvp("ros2", c_args.data());
        _exit(1);
    } else if (playback_pid_ < 0) {
        status_msg_ = "Error: Failed to fork";
        playback_pid_ = -1;
    } else {
        is_paused_ = start_paused;
        finished_ = false;
        node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
        clock_sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::SensorDataQoS(), std::bind(&RosbagDisplay::clock_callback, this, std::placeholders::_1));
        
        if (start_paused) status_msg_ = "Paused at start";
        else status_msg_ = "Playing (" + std::to_string(playback_rate_).substr(0,3) + "x)";
    }
}

void RosbagDisplay::stop_playback() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (playback_pid_ > 0) {
        kill(-playback_pid_, SIGINT);
        int status;
        for (int i = 0; i < 10; ++i) {
            if (waitpid(playback_pid_, &status, WNOHANG) > 0) break;
            usleep(100000);
        }
    }
    playback_pid_ = -1;
    finished_ = false;
    is_paused_ = false;
    clock_sub_.reset();
    node_->set_parameter(rclcpp::Parameter("use_sim_time", false));
    status_msg_ = "Stopped.";
}

void RosbagDisplay::render(RvizRenderer&, ftxui::Canvas&, const std::string&, std::shared_ptr<tf2_ros::Buffer>) {}

ftxui::Element RosbagDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);

    // Check for natural process exit (finished playing)
    if (playback_pid_ > 0) {
        int status;
        if (waitpid(playback_pid_, &status, WNOHANG) > 0) {
            playback_pid_ = -1;
            finished_ = true;
            status_msg_ = "Finished (Rewind available)";
            current_progress_ = 1.0f;
        }
    }

    auto tab_record = text(" RECORD ") | (tab_idx_ == 0 ? inverted | color(Color::Red) : nothing);
    auto tab_play = text(" PLAY ") | (tab_idx_ == 1 ? inverted | color(Color::Green) : nothing);

    Elements content;
    if (tab_idx_ == 0) {
        content.push_back(hbox({ text(" Save To: ") | bold, text(output_path_) | color(Color::Cyan) | flex, text(" [CHANGE]") | color(Color::GrayLight) }));
        content.push_back(separator());
        
        Elements topics_ui;
        auto topic_names = node_->get_topic_names_and_types();
        std::vector<std::string> sorted_topics;
        for (auto const& [name, types] : topic_names) sorted_topics.push_back(name);
        std::sort(sorted_topics.begin(), sorted_topics.end());

        int visible_count = 0;
        int max_visible = 4;
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
        while (topics_ui.size() < (size_t)max_visible) topics_ui.push_back(filler());
        content.push_back(vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 4));
        
        auto start_btn = text(" [ START ] ") | border | (recording_pid_ > 0 ? dim : bold | color(Color::Red));
        auto stop_btn = text(" [ STOP ] ") | border | (recording_pid_ <= 0 ? dim : bold | color(Color::White) | inverted);
        content.push_back(separator());
        content.push_back(hbox({ filler(), start_btn, stop_btn, filler() }));
    } else {
        content.push_back(hbox({ text(" Bag File: ") | bold, text(input_path_) | color(Color::Cyan) | flex, text(" [SELECT]") | color(Color::GrayLight) }));
        content.push_back(separator());
        
        auto rate_info = hbox({
            text(" Rate: ") | bold,
            text(" [-] ") | color(Color::Yellow),
            text(std::to_string(playback_rate_).substr(0, 3) + "x") | color(Color::Green) | bold,
            text(" [+] ") | color(Color::Yellow),
            filler(),
            text(is_looping_ ? " [X] LOOP " : " [ ] LOOP ") | color(is_looping_ ? Color::Green : Color::GrayDark),
            filler(),
            text(is_paused_ ? " [RESUME] " : " [PAUSE] ") | color(is_paused_ ? (Color)Color::Green : (Color)Color::Orange1) | inverted
        });
        content.push_back(rate_info);
        content.push_back(separator());
        
        long long current_sec = (long long)(current_progress_ * (bag_duration_ns_ / 1000000000LL));
        auto format_time = [](long long s) {
            int mins = s / 60;
            int secs = s % 60;
            std::stringstream ss;
            ss << std::setfill('0') << std::setw(2) << mins << ":" << std::setw(2) << secs;
            return ss.str();
        };

        content.push_back(hbox({
            text(" Seek: ") | bold,
            gauge(current_progress_) | color(Color::Blue) | flex,
            text(" " + format_time(current_sec) + " (" + std::to_string((int)(current_progress_ * 100)) + "%)") | dim
        }));

        auto start_btn = text(" [ PLAY ] ") | border | (playback_pid_ > 0 ? dim : bold | color(Color::Green));
        auto stop_btn = text(" [ STOP ] ") | border | ((playback_pid_ <= 0 && !finished_) ? dim : bold | color(Color::White) | inverted);
        content.push_back(separator());
        content.push_back(hbox({ filler(), start_btn, stop_btn, filler() }));
    }

    return vbox({
        hbox({ text(" Rosbag ") | bold | color(Color::Yellow), filler(), tab_record, separator(), tab_play }),
        separator(),
        vbox(std::move(content)) | flex,
        separator(),
        text(" Status: " + status_msg_) | dim
    }) | border | size(HEIGHT, EQUAL, 14);
}

bool RosbagDisplay::handle_event(ftxui::Event, int) { return false; }

} // namespace terminal_rviz
