#include "CUSB_RAD121.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <deque>

class Rad121MonitorNode : public rclcpp::Node
{
public:
    Rad121MonitorNode() : Node("rad121_monitor_node"), decayRate(0.99), updateInterval(1)
    {
        rad121_ = std::make_shared<CUSB_RAD121>();
        if (!rad121_->Open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open FTDI device");
            rclcpp::shutdown();
        }
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rad", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / updateInterval), std::bind(&Rad121MonitorNode::MonitorClicks, this));
    }

    ~Rad121MonitorNode()
    {
        rad121_->Close();
    }

private:
    void MonitorClicks()
    {
        unsigned char buffer[64]; // Read a chunk of data
        auto currentTime = std::chrono::steady_clock::now();

        int count = rad121_->ReadData(buffer, sizeof(buffer));
        if (count < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading data from FTDI device");
            rclcpp::shutdown();
        }

        if (count > 0)
        {
            for (int i = 0; i < count; i++)
            {
                // Record the timestamp for each count
                countTimestamps.push_back(currentTime);
            }
        }

        // Remove timestamps older than one second
        while (!countTimestamps.empty() && std::chrono::duration_cast<std::chrono::seconds>(currentTime - countTimestamps.front()).count() >= 1)
        {
            countTimestamps.pop_front();
        }

        // Calculate CPS (Counts Per Second)
        double cps = countTimestamps.size();
        // Calculate CPM (Counts Per Minute)
        double cpm = cps * 60.0;
        // Calculate Compensated CPM
        double cmp_compensated = rad121_->Calculate_CompensatedCPM(cpm);
        // Calculate mR/hr
        double mR_hr = rad121_->Calculate_mRhr(cmp_compensated);

        // Publish the values
        auto message = std_msgs::msg::Float64MultiArray();
        message.data.push_back(cmp_compensated); // Compensated CPM
        message.data.push_back(mR_hr);          // mR/hr
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Compensated CPM: %.2f | mR/hr: %.6f", cmp_compensated, mR_hr);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std::shared_ptr<CUSB_RAD121> rad121_;
    std::deque<std::chrono::steady_clock::time_point> countTimestamps;
    const double decayRate;
    const int updateInterval;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Rad121MonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

