// TOSM semantic mission executor, BT.CPP 4 port of semantic_nav_bt.
//
// Same hybrid deliberative/reactive tree as the py_trees implementation
// (paper Fig. 16), but executed with BehaviorTree.CPP so a live Groot2
// instance can attach through Groot2Publisher (default port 1667).
// Semantic resolution stays in Python: leaves call the /resolve_semantic
// service served by `ros2 run semantic_nav_bt mediator_server`, which keeps
// the knowledge-graph hot-reload (owner edits in the web UI) intact.
//
// Topics kept wire-compatible with the Python executor so the web console
// works unchanged: /semantic_command, /battery_state in;
// /semantic_status, /bt_snapshot out.

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/contrib/json.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "semantic_nav_msgs/srv/resolve_semantic.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

using json = nlohmann::json;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ResolveSemantic = semantic_nav_msgs::srv::ResolveSemantic;

// ----------------------------------------------------------------- context
// Shared mission state (the "blackboard" of the Python port — a plain
// struct shared by construction keeps the leaves symmetrical with
// bt_nodes.py and avoids stringly-typed BT ports for internal state).
struct MissionContext
{
  std::mutex mtx;
  std::deque<json> pending;
  json command;                 // currently executing command (null if none)
  std::vector<json> goals;
  size_t goal_idx = 0;
  bool home_requested = false;
  bool stop_requested = false;   // operator STOP: cancel nav, hold position
  bool docked = false;
  double battery = 1.0;
  bool have_pose = false;
  double px = 0.0, py = 0.0;
  std::string status = "idle";

  void set_status(const std::string & s) {status = s;}
  void clear_mission()
  {
    command = nullptr;
    goals.clear();
    goal_idx = 0;
  }
};
using Ctx = std::shared_ptr<MissionContext>;

// -------------------------------------------------------------- conditions
class BatteryOK : public BT::ConditionNode
{
public:
  BatteryOK(const std::string & name, const BT::NodeConfig & cfg, Ctx ctx)
  : BT::ConditionNode(name, cfg), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    if (ctx_->battery < threshold_) {
      if (ctx_->docked) {
        char b[64];
        snprintf(b, sizeof(b), "low battery, holding at home (%.2f)",
                 ctx_->battery);
        ctx_->set_status(b);
      } else {
        char b[80];
        snprintf(b, sizeof(b), "battery %.2f < %.2f -> return home",
                 ctx_->battery, threshold_);
        ctx_->set_status(b);
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  Ctx ctx_;
  double threshold_ = 0.2;
};

class NoHomeInterrupt : public BT::ConditionNode
{
public:
  NoHomeInterrupt(const std::string & name, const BT::NodeConfig & cfg,
                  Ctx ctx)
  : BT::ConditionNode(name, cfg), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    return ctx_->home_requested ? BT::NodeStatus::FAILURE :
           BT::NodeStatus::SUCCESS;
  }

private:
  Ctx ctx_;
};

// ------------------------------------------------------------------ leaves
class WaitForCommand : public BT::StatefulActionNode
{
public:
  WaitForCommand(const std::string & name, const BT::NodeConfig & cfg,
                 Ctx ctx)
  : BT::StatefulActionNode(name, cfg), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override {return step();}
  BT::NodeStatus onRunning() override {return step();}
  void onHalted() override {}

private:
  BT::NodeStatus step()
  {
    std::lock_guard<std::mutex> lk(ctx_->mtx);
    if (!ctx_->command.is_null()) {return BT::NodeStatus::SUCCESS;}
    ctx_->stop_requested = false;                  // nothing to stop while idle
    if (!ctx_->pending.empty()) {
      ctx_->command = ctx_->pending.front();
      ctx_->pending.pop_front();
      ctx_->docked = false;
      ctx_->goals.clear();
      ctx_->goal_idx = 0;
      ctx_->set_status("command: " + ctx_->command.dump());
      return BT::NodeStatus::SUCCESS;
    }
    ctx_->set_status("idle (waiting for /semantic_command)");
    return BT::NodeStatus::RUNNING;
  }
  Ctx ctx_;
};

// Resolve through the Python mediator service (KG stays in Python).
class ResolveSemanticGoal : public BT::StatefulActionNode
{
public:
  ResolveSemanticGoal(const std::string & name, const BT::NodeConfig & cfg,
                      Ctx ctx, rclcpp::Node::SharedPtr node,
                      json fixed_command = nullptr)
  : BT::StatefulActionNode(name, cfg), ctx_(ctx), node_(node),
    fixed_(fixed_command)
  {
    client_ = node_->create_client<ResolveSemantic>("resolve_semantic");
  }
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    if (fixed_.is_null() && !ctx_->goals.empty()) {
      return BT::NodeStatus::SUCCESS;      // continuing a multi-goal mission
    }
    if (!client_->service_is_ready()) {
      ctx_->set_status("mediator service unavailable");
      return BT::NodeStatus::FAILURE;
    }
    auto req = std::make_shared<ResolveSemantic::Request>();
    req->command_json =
      (fixed_.is_null() ? ctx_->command : fixed_).dump();
    future_ = client_->async_send_request(req).future.share();
    deadline_ = node_->now() + rclcpp::Duration::from_seconds(5.0);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (future_.wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready)
    {
      if (node_->now() > deadline_) {
        ctx_->set_status("mediator timeout");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    json r = json::parse(future_.get()->result_json, nullptr, false);
    if (r.is_discarded() || r.contains("error")) {
      std::string err = r.is_discarded() ? "bad mediator reply"
                                         : r["error"].get<std::string>();
      ctx_->set_status("mediator REFUSED: " + err);
      std::lock_guard<std::mutex> lk(ctx_->mtx);
      ctx_->clear_mission();
      return BT::NodeStatus::FAILURE;
    }
    std::lock_guard<std::mutex> lk(ctx_->mtx);
    ctx_->goals = r["goals"].get<std::vector<json>>();
    ctx_->goal_idx = 0;
    std::string labels;
    for (const auto & g : ctx_->goals) {
      labels += (labels.empty() ? "" : ", ") + g["label"].get<std::string>();
    }
    ctx_->set_status("resolved " + std::to_string(ctx_->goals.size()) +
                     " goal(s): [" + labels + "]");
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {}

private:
  Ctx ctx_;
  rclcpp::Node::SharedPtr node_;
  json fixed_;
  rclcpp::Client<ResolveSemantic>::SharedPtr client_;
  std::shared_future<std::shared_ptr<ResolveSemantic::Response>> future_;
  rclcpp::Time deadline_;
};

class NavigateToGoal : public BT::StatefulActionNode
{
public:
  NavigateToGoal(const std::string & name, const BT::NodeConfig & cfg,
                 Ctx ctx, rclcpp::Node::SharedPtr node, bool dry_run)
  : BT::StatefulActionNode(name, cfg), ctx_(ctx), node_(node),
    dry_run_(dry_run)
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(
      node_, "navigate_to_pose");
  }
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    result_.clear();
    ticks_ = 0;
    handle_.reset();
    if (ctx_->goal_idx >= ctx_->goals.size()) {return BT::NodeStatus::FAILURE;}
    const json & g = ctx_->goals[ctx_->goal_idx];
    // already there (e.g. holding at home): succeed without a nav goal
    if (g.value("kind", "") == "return_home" && ctx_->have_pose) {
      double dx = ctx_->px - g["x"].get<double>();
      double dy = ctx_->py - g["y"].get<double>();
      if (dx * dx + dy * dy < 0.3 * 0.3) {
        result_ = "succeeded";
        return BT::NodeStatus::RUNNING;
      }
    }
    if (dry_run_) {return BT::NodeStatus::RUNNING;}
    if (!client_->action_server_is_ready()) {
      ctx_->set_status("Nav2 action server unavailable");
      return skip_goal();
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = g["x"].get<double>();
    goal.pose.pose.position.y = g["y"].get<double>();
    double yaw = g.value("yaw", 0.0);
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
    auto opts =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr h) {
        handle_ = h;
        if (!h) {result_ = "rejected";}
      };
    opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::
             WrappedResult & r) {
        result_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED) ?
          "succeeded" : "ended(" + std::to_string(int(r.code)) + ")";
      };
    client_->async_send_goal(goal, opts);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    {
      std::lock_guard<std::mutex> lk(ctx_->mtx);
      if (ctx_->stop_requested) {
        if (handle_ && result_.empty()) {client_->async_cancel_goal(handle_);}
        ctx_->stop_requested = false;
        ctx_->clear_mission();
        ctx_->set_status("STOPPED by operator: mission cancelled, holding position");
        return BT::NodeStatus::FAILURE;
      }
      if (ctx_->goal_idx >= ctx_->goals.size()) {   // mission cleared under us
        if (handle_ && result_.empty()) {client_->async_cancel_goal(handle_);}
        return BT::NodeStatus::FAILURE;
      }
    }
    const json & g = ctx_->goals[ctx_->goal_idx];
    char b[160];
    if (dry_run_) {
      snprintf(b, sizeof(b), "[dry] navigating to %s (%.2f,%.2f) t=%d",
               g["label"].get<std::string>().c_str(),
               g["x"].get<double>(), g["y"].get<double>(), ++ticks_);
      ctx_->set_status(b);
      return ticks_ >= 3 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }
    if (result_.empty()) {
      snprintf(b, sizeof(b), "navigating to %s (%.2f,%.2f)",
               g["label"].get<std::string>().c_str(),
               g["x"].get<double>(), g["y"].get<double>());
      ctx_->set_status(b);
      return BT::NodeStatus::RUNNING;
    }
    ctx_->set_status("nav " + result_ + ": " +
                     g["label"].get<std::string>());
    if (result_ == "succeeded") {return BT::NodeStatus::SUCCESS;}
    return skip_goal();
  }

  void onHalted() override
  {
    if (handle_ && result_.empty()) {client_->async_cancel_goal(handle_);}
  }

private:
  BT::NodeStatus skip_goal()
  {
    // log + skip the failed goal so the mission loop moves on (mirrors
    // the Python executor; going home stays reserved for battery/interrupt)
    RCLCPP_WARN(node_->get_logger(), "goal failed (%s), skipping",
                result_.c_str());
    std::lock_guard<std::mutex> lk(ctx_->mtx);
    ctx_->goal_idx++;
    if (ctx_->goal_idx >= ctx_->goals.size()) {ctx_->clear_mission();}
    return BT::NodeStatus::FAILURE;
  }

  Ctx ctx_;
  rclcpp::Node::SharedPtr node_;
  bool dry_run_;
  int ticks_ = 0;
  std::string result_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr handle_;
};

// Manipulator stub — the seam where the real AMMR arm plugs in (thesis).
class ArmPick : public BT::StatefulActionNode
{
public:
  ArmPick(const std::string & name, const BT::NodeConfig & cfg, Ctx ctx)
  : BT::StatefulActionNode(name, cfg), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    ticks_ = 0;
    return step();
  }
  BT::NodeStatus onRunning() override {return step();}
  void onHalted() override {}

private:
  BT::NodeStatus step()
  {
    const json & g = ctx_->goals[ctx_->goal_idx];
    if (g.value("kind", "") != "pick") {return BT::NodeStatus::SUCCESS;}
    const json & t = g["pick_target"];
    if (t.contains("isMovable") && t["isMovable"].is_boolean() &&
        !t["isMovable"].get<bool>())
    {
      ctx_->set_status("pick REFUSED: " + t["name"].get<std::string>() +
                       " isMovable=false");
      std::lock_guard<std::mutex> lk(ctx_->mtx);
      ctx_->goal_idx++;
      if (ctx_->goal_idx >= ctx_->goals.size()) {ctx_->clear_mission();}
      return BT::NodeStatus::FAILURE;
    }
    ticks_++;
    ctx_->set_status("[arm stub] picking " + t["name"].get<std::string>() +
                     " " + std::to_string(ticks_) + "/5");
    return ticks_ >= 5 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  Ctx ctx_;
  int ticks_ = 0;
};

class AdvanceGoal : public BT::SyncActionNode
{
public:
  AdvanceGoal(const std::string & name, const BT::NodeConfig & cfg, Ctx ctx)
  : BT::SyncActionNode(name, cfg), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mtx);
    ctx_->goal_idx++;
    if (ctx_->goal_idx >= ctx_->goals.size()) {
      ctx_->set_status("mission complete: " + ctx_->command.dump());
      ctx_->clear_mission();
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  Ctx ctx_;
};

class HomeDone : public BT::SyncActionNode
{
public:
  HomeDone(const std::string & name, const BT::NodeConfig & cfg, Ctx ctx)
  : BT::SyncActionNode(name, cfg), ctx_(ctx) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mtx);
    ctx_->home_requested = false;
    ctx_->docked = true;
    ctx_->clear_mission();
    ctx_->set_status("at home");
    return BT::NodeStatus::SUCCESS;
  }

private:
  Ctx ctx_;
};

// ------------------------------------------------------------ mission node
class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : rclcpp::Node("semantic_mission_bt_cpp")
  {
    ctx_ = std::make_shared<MissionContext>();
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "semantic_command", 10,
      [this](const std_msgs::msg::String & m) {on_command(m);});
    batt_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", 10,
      [this](const sensor_msgs::msg::BatteryState & m) {
        if (m.percentage > 0.0f) {
          ctx_->battery = m.percentage > 1.0f ? m.percentage / 100.0 :
            m.percentage;
        }
      });
    auto amcl_qos = rclcpp::QoS(1).reliable().transient_local();
    pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", amcl_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped & m) {
        ctx_->px = m.pose.pose.position.x;
        ctx_->py = m.pose.pose.position.y;
        ctx_->have_pose = true;
      });
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "semantic_status", 10);
    snapshot_pub_ = create_publisher<std_msgs::msg::String>(
      "bt_snapshot", 10);
  }

  Ctx ctx() {return ctx_;}

  void publish_state(const BT::Tree & tree)
  {
    json st = {{"status", ctx_->status},
      {"command", ctx_->command},
      {"goal_idx", ctx_->goal_idx},
      {"n_goals", ctx_->goals.size()},
      {"battery", ctx_->battery},
      {"home_requested", ctx_->home_requested}};
    std_msgs::msg::String m;
    m.data = st.dump();
    status_pub_->publish(m);
    std_msgs::msg::String snap;
    snap.data = snapshot(tree.rootNode()).dump();
    snapshot_pub_->publish(snap);
  }

private:
  void on_command(const std_msgs::msg::String & m)
  {
    json cmd = json::parse(m.data, nullptr, false);
    if (cmd.is_discarded()) {                       // bare-word convenience
      std::string w = m.data;
      if (w == "home" || w == "dock") {cmd = {{"cmd", "return_home"}};}
      else if (w.rfind("goto ", 0) == 0) {
        cmd = {{"cmd", "goto_object"}, {"target", w.substr(5)}};
      } else {
        RCLCPP_WARN(get_logger(), "bad command: %s", m.data.c_str());
        return;
      }
    }
    RCLCPP_INFO(get_logger(), "command: %s", cmd.dump().c_str());
    std::lock_guard<std::mutex> lk(ctx_->mtx);
    if (cmd.value("cmd", "") == "stop" || cmd.value("cmd", "") == "cancel") {
      ctx_->pending.clear();
      ctx_->home_requested = false;
      if (!ctx_->command.is_null()) {
        ctx_->stop_requested = true;               // NavigateToGoal cancels nav
      } else {
        ctx_->set_status("STOP: no mission running");
      }
      return;
    }
    if (cmd.value("cmd", "") == "return_home" ||
        cmd.value("cmd", "") == "dock")
    {
      ctx_->home_requested = true;
    } else {
      ctx_->pending.push_back(cmd);
    }
  }

  // same JSON schema as the py_trees executor, so the web UI's
  // Groot-style tab renders either backend unchanged
  json snapshot(const BT::TreeNode * n)
  {
    std::string st = BT::toStr(n->status());
    if (st == "IDLE" || st == "SKIPPED") {st = "INVALID";}
    std::string kind = "composite";
    switch (n->type()) {
      case BT::NodeType::CONDITION: kind = "condition"; break;
      case BT::NodeType::ACTION: kind = "action"; break;
      case BT::NodeType::DECORATOR: kind = "decorator"; break;
      default: break;
    }
    json out = {{"name", n->name()},
      {"cls", n->registrationName()},
      {"kind", kind}, {"status", st},
      {"children", json::array()}};
    if (auto ctrl = dynamic_cast<const BT::ControlNode *>(n)) {
      for (const auto * c : ctrl->children()) {
        out["children"].push_back(snapshot(c));
      }
    } else if (auto dec = dynamic_cast<const BT::DecoratorNode *>(n)) {
      out["children"].push_back(snapshot(dec->child()));
    }
    return out;
  }

  Ctx ctx_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
  SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr snapshot_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionNode>();
  auto ctx = node->ctx();

  bool dry_run = node->declare_parameter("dry_run", false);
  double tick_hz = node->declare_parameter("tick_hz", 2.0);
  int groot2_port = node->declare_parameter("groot2_port", 1667);
  std::string default_xml =
    ament_index_cpp::get_package_share_directory("semantic_nav_bt_cpp") +
    "/trees/semantic_mission.xml";
  std::string xml = node->declare_parameter("tree_xml", default_xml);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<BatteryOK>("BatteryOK", ctx);
  factory.registerNodeType<NoHomeInterrupt>("NoHomeInterrupt", ctx);
  factory.registerNodeType<WaitForCommand>("WaitForCommand", ctx);
  factory.registerNodeType<ResolveSemanticGoal>(
    "ResolveSemanticGoal", ctx, node, json(nullptr));
  factory.registerNodeType<ResolveSemanticGoal>(
    "ResolveHome", ctx, node, json({{"cmd", "return_home"}}));
  factory.registerNodeType<NavigateToGoal>(
    "NavigateToGoal", ctx, node, dry_run);
  factory.registerNodeType<ArmPick>("ArmPick", ctx);
  factory.registerNodeType<AdvanceGoal>("AdvanceGoal", ctx);
  factory.registerNodeType<HomeDone>("HomeDone", ctx);

  auto tree = factory.createTreeFromFile(xml);
  BT::Groot2Publisher groot2(tree, groot2_port);

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(int(1000.0 / tick_hz)),
    [&]() {
      tree.tickExactlyOnce();
      node->publish_state(tree);
    });

  RCLCPP_INFO(node->get_logger(),
              "semantic mission BT.CPP up (dry_run=%d, Groot2 port %d, %s)",
              int(dry_run), groot2_port, xml.c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
