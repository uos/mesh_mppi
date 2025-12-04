#include <mbf_mesh_core/mesh_controller.h>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <mesh_mppi/scoring/CostFunction.hpp>
#include <mesh_mppi/kinematics/DifferentialDrive.hpp>
#include <mesh_mppi/optimizer/AsyncOptimizer.hpp>
#include <mesh_mppi/types/Velocity.hpp>

#include <mesh_mppi_msgs/msg/control_sequence_stamped.hpp>

namespace mesh_mppi
{

using mesh_mppi_msgs::msg::ControlSequenceStamped;

class MeshMPPIBase: public mbf_mesh_core::MeshController
{
public:
    typedef std::shared_ptr<MeshMPPIBase> Ptr;

    enum class StateMachine;
    
    // TODO: Proper initialization of all fields
    MeshMPPIBase()
    : logger_(rclcpp::get_logger("MeshMPPIController"))
    {};
    
    /**
    * @brief Destructor
    */
    virtual ~MeshMPPIBase() = default;


    /**
    * @brief Given the current position, orientation, and velocity of the robot,
    * compute velocity commands to send to the base.
    * @param pose The current pose of the robot.
    * @param velocity The current velocity of the robot.
    * @param cmd_vel Will be filled with the velocity command to be passed to the
    * robot base.
    * @param message Optional more detailed outcome as a string
    * @return Result code as described on ExePath action result (see ExePath.action)
    */
    virtual uint32_t computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::TwistStamped& velocity,
        geometry_msgs::msg::TwistStamped& cmd_vel,
        std::string& message
    ) override = 0;

    /**
    * @brief Check if the goal pose has been achieved by the local planner
    * @param angle_tolerance The angle tolerance in which the current pose will
    * be partly accepted as reached goal
    * @param dist_tolerance The distance tolerance in which the current pose will
    * be partly accepted as reached goal
    * @return True if achieved, false otherwise
    */
    virtual bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

    /**
    * @brief Set the plan that the local planner is following
    * @param plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
    virtual bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan) override;

    /**
    * @brief Requests the planner to cancel, e.g. if it takes too much time.
    * @return True if a cancel has been successfully requested, false if not
    * implemented.
    * TODO: It would be nice to gracefully stop the robot when a request to cancel
    * is received. My idea is have a simple state machine in the computeVelocityCommands
    * method and switch the cost function target to a 0 velocity.
    */
    virtual bool cancel() override {return false;};

    /**
    * @brief Initializes the controller plugin with a name, a tf pointer and a mesh map pointer
    * @param plugin_name The controller plugin name, defined by the user. It defines the controller namespace
    * @param tf_ptr A shared pointer to a transformation buffer
    * @param mesh_map_ptr A shared pointer to the mesh map
    * @return true if the plugin has been initialized successfully
    */
    virtual bool initialize(
        const std::string& name,
        const std::shared_ptr<tf2_ros::Buffer>& tf_ptr,
        const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr,
        const rclcpp::Node::SharedPtr& node
    ) override;

protected:
    // Initialize the Kinematics and optimizer
    virtual bool initialize() = 0;

    virtual OptimizerBase& getOptimizerBase() = 0;

    virtual KinematicsBase& getKinematicsBase() = 0;

    // Needed so the MeshMPPIBase class can reset the MeshMPPI future instance
    virtual void resetFuture() = 0;

    inline rclcpp::Logger& getLogger()
    {
        return logger_;
    }

    [[nodiscard]] bool updateCurrentFace(const State& current);

    void setCurrentPoseAndVelocity(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Twist& vel);

    lvr2::OptionalFaceHandle getCurrentFace() const;

    [[nodiscard]] bool isMakingProgress(const Trajectory& traj, double cost);


    void publishOptimalTrajectory(const Trajectory& traj);

    void publishOptimalControlSequence(const Eigen::ArrayXf& data, uint32_t timesteps, uint32_t signals, const rclcpp::Time& timestamp);

    // Configuration of the controller plugin
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<mesh_map::MeshMap> map_;
    rclcpp::Node::SharedPtr node_;

    // The current parameters of the controller
    struct {
        int horizon;
        int samples;
        float map_cost_limit;
        float controller_frequency;
    } params_;

    // The cost function
    std::shared_ptr<CostFunction> cost_function_;

    // The current state of the controller
    StateMachine state_;

private:

    [[nodiscard]] bool initializeParameters();

    rcl_interfaces::msg::SetParametersResult reconfigure(const std::vector<rclcpp::Parameter>& parameters);

    // Reconfigure callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr reconfigure_callback_handle_;

    // The last known pose
    struct {
        bool valid;
        Eigen::Vector3f position;
        Eigen::Quaternionf orientation;
        Velocity velocity;
        lvr2::OptionalFaceHandle faceH;
    } pose_;

    // Data for progress checking
    std::deque<mesh_map::Vector> past_trajectory_;
    std::deque<double> past_costs_;
    double progress_translation_threshold_;
    double progress_cost_reduction_threshold_;
    double progress_num_timesteps_;

    // The current plan
    std::vector<geometry_msgs::msg::PoseStamped> plan_;

    // A publisher for the best trajectory
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    rclcpp::Publisher<ControlSequenceStamped>::SharedPtr sequence_pub_;

    // A logger instance for this controller. Use getLogger() to access the logger instance
    rclcpp::Logger logger_;
};


enum class MeshMPPIBase::StateMachine {
    IDLE,
    MOVING,
    REACHED_GOAL,
    FAILED_GOAL
};

} // namespace mesh_mppi
