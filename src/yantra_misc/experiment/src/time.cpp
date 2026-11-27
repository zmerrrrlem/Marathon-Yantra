    #include <chrono>
    #include <string>

    #include <rclcpp/rclcpp.hpp>

    using namespace std::chrono_literals;

    class TestClockNode : public rclcpp::Node
    {
    public:
        double pasts;
        rclcpp::Time past;
        rclcpp::Time pastx;

        TestClockNode(
            const std::string & node_name,
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) :
        rclcpp::Node{node_name, options}
        {
        timer_ = create_wall_timer(1s, [this] () {timerCallback();});
        }

    private:

        rclcpp::TimerBase::SharedPtr timer_;
        void timerCallback()
        {
            if (rclcpp::Clock{}.now().seconds() > pasts) {
                rclcpp::Duration D = past - rclcpp::Clock{RCL_ROS_TIME}.now();

                rclcpp::Duration DD = pastx - rclcpp::Clock().now();
                RCLCPP_INFO_STREAM(get_logger(), "inside if " << rclcpp::Clock{}.now().seconds() - pasts);
                RCLCPP_INFO_STREAM(get_logger(), "inside if " << D.nanoseconds());
                RCLCPP_INFO_STREAM(get_logger(), "inside if " << DD.nanoseconds());
            } 
            RCLCPP_INFO_STREAM(get_logger(), "now(): " << now().seconds());
            RCLCPP_INFO_STREAM(get_logger(), "  rclcpp::Clock{}.now(): " << rclcpp::Clock{}.now().seconds());
            RCLCPP_INFO_STREAM(get_logger(), "  rclcpp::Clock{RCL_ROS_TIME}.now(): " << rclcpp::Clock{RCL_ROS_TIME}.now().seconds());
            RCLCPP_INFO_STREAM(get_logger(), "  rclcpp::Clock{RCL_SYSTEM_TIME}.now(): " << rclcpp::Clock{RCL_SYSTEM_TIME}.now().seconds());
            RCLCPP_INFO_STREAM(get_logger(), "  rclcpp::Clock{RCL_STEADY_TIME}.now().seconds: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds());
            RCLCPP_INFO_STREAM(get_logger(), "  rclcpp::Clock{RCL_STEADY_TIME}.now().nanoseconds(): " << rclcpp::Clock{RCL_STEADY_TIME}.now().nanoseconds());
        }

    };

    int
    main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TestClockNode>("test_clock");
    node->pasts = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();
    node->past = rclcpp::Clock{RCL_ROS_TIME}.now();
    node->pastx = rclcpp::Clock().now();
    if (node == nullptr)
    {
        return 1;
    }

    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    executor.spin();

    

    rclcpp::shutdown();

    return 0;
    }