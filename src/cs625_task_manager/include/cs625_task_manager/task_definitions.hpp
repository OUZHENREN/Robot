#ifndef CS625_TASK_MANAGER__TASK_DEFINITIONS_HPP_
#define CS625_TASK_MANAGER__TASK_DEFINITIONS_HPP_

#include <cstdint>
#include <string>

namespace cs625_task_manager
{

namespace constants
{

// Step IDs
static constexpr uint32_t STEP_IDLE = 0;

// Load flow: E0 ~ E9
static constexpr uint32_t STEP_E0_RESET_IO = 1;
static constexpr uint32_t STEP_E1_PRE_GRASP = 2;
static constexpr uint32_t STEP_E2_ENTER_GRASP = 3;
static constexpr uint32_t STEP_E3_GRASP = 4;
static constexpr uint32_t STEP_E4_TRANSIT = 5;
static constexpr uint32_t STEP_E5_PRE_INSERT = 6;
static constexpr uint32_t STEP_E6_PRE_INSERT_ROTATED = 7;
static constexpr uint32_t STEP_E7_FINAL_INSERT = 8;
static constexpr uint32_t STEP_E8_RELEASE = 9;
static constexpr uint32_t STEP_E9_RETREAT = 10;

// Unload flow: D0 ~ D9
static constexpr uint32_t STEP_D0_RESET_IO = 101;
static constexpr uint32_t STEP_D1_PRE_GRASP = 102;
static constexpr uint32_t STEP_D2_ENTER_GRASP = 103;
static constexpr uint32_t STEP_D3_GRASP = 104;
static constexpr uint32_t STEP_D4_PRE_REMOVE_ROTATED = 105;
static constexpr uint32_t STEP_D5_REMOVE = 106;
static constexpr uint32_t STEP_D6_TRANSIT = 107;
static constexpr uint32_t STEP_D7_PLACE = 108;
static constexpr uint32_t STEP_D8_RELEASE = 109;
static constexpr uint32_t STEP_D9_RETREAT = 110;

static constexpr uint32_t STEP_HOME_RETURN = 1000;

// High-level phases
static const std::string PHASE_IDLE = "IDLE";
static const std::string PHASE_VISION = "VISION";
static const std::string PHASE_EXECUTION = "EXECUTION";
static const std::string PHASE_DONE = "DONE";
static const std::string PHASE_ERROR = "ERROR";

// Status values
static const std::string STATUS_IDLE = "IDLE";
static const std::string STATUS_READY = "READY";
static const std::string STATUS_STEP_TRIGGERED = "STEP_TRIGGERED";
static const std::string STATUS_RUNNING = "RUNNING";
static const std::string STATUS_SUCCESS = "SUCCESS";
static const std::string STATUS_REJECTED = "REJECTED";
static const std::string STATUS_ERROR = "ERROR";
static const std::string STATUS_WAITING_CONFIRMATION = "WAITING_CONFIRMATION";
static const std::string STATUS_COMPLETED = "COMPLETED";

// Execution substates
static const std::string EXEC_SUBSTATE_NONE = "NONE";
static const std::string EXEC_SUBSTATE_WAITING_TRIGGER = "WAITING_TRIGGER";
static const std::string EXEC_SUBSTATE_IO_RESET_DONE = "IO_RESET_DONE";
static const std::string EXEC_SUBSTATE_PRE_GRASP_REACHED = "PRE_GRASP_REACHED";
static const std::string EXEC_SUBSTATE_GRASP_POSE_REACHED = "GRASP_POSE_REACHED";
static const std::string EXEC_SUBSTATE_WAITING_GRASP_CONFIRM = "WAITING_GRASP_CONFIRM";
static const std::string EXEC_SUBSTATE_BOX_GRASPED = "BOX_GRASPED";

static const std::string EXEC_SUBSTATE_TRANSIT_REACHED_WAITING_STEP_CONFIRM =
  "TRANSIT_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_TRANSIT_STEP_CONFIRMED =
  "TRANSIT_STEP_CONFIRMED";

static const std::string EXEC_SUBSTATE_PRE_INSERT_REACHED_WAITING_STEP_CONFIRM =
  "PRE_INSERT_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_PRE_INSERT_STEP_CONFIRMED =
  "PRE_INSERT_STEP_CONFIRMED";

static const std::string EXEC_SUBSTATE_PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM =
  "PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_PRE_INSERT_ROTATED_STEP_CONFIRMED =
  "PRE_INSERT_ROTATED_STEP_CONFIRMED";

static const std::string EXEC_SUBSTATE_INSERT_POSE_REACHED_WAITING_STEP_CONFIRM =
  "INSERT_POSE_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_INSERT_STEP_CONFIRMED =
  "INSERT_STEP_CONFIRMED";

static const std::string EXEC_SUBSTATE_PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM =
  "PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_PRE_REMOVE_ROTATED_STEP_CONFIRMED =
  "PRE_REMOVE_ROTATED_STEP_CONFIRMED";

static const std::string EXEC_SUBSTATE_REMOVE_REACHED_WAITING_STEP_CONFIRM =
  "REMOVE_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_REMOVE_STEP_CONFIRMED =
  "REMOVE_STEP_CONFIRMED";

static const std::string EXEC_SUBSTATE_PLACE_POSE_REACHED_WAITING_STEP_CONFIRM =
  "PLACE_POSE_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_PLACE_STEP_CONFIRMED =
  "PLACE_STEP_CONFIRMED";

static const std::string EXEC_SUBSTATE_WAITING_RELEASE_CONFIRM = "WAITING_RELEASE_CONFIRM";
static const std::string EXEC_SUBSTATE_BOX_RELEASED = "BOX_RELEASED";

static const std::string EXEC_SUBSTATE_RETREAT_REACHED_WAITING_STEP_CONFIRM =
  "RETREAT_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_RETREAT_STEP_CONFIRMED =
  "RETREAT_STEP_CONFIRMED";

static const std::string EXEC_SUBSTATE_FINISHED = "FINISHED";

static const std::string EXEC_SUBSTATE_HOME_REACHED_WAITING_STEP_CONFIRM =
  "HOME_REACHED_WAITING_STEP_CONFIRM";
static const std::string EXEC_SUBSTATE_HOME_STEP_CONFIRMED =
  "HOME_STEP_CONFIRMED";

// Box state
static const std::string BOX_STATE_UNKNOWN = "UNKNOWN";
static const std::string BOX_STATE_NOT_GRASPED = "NOT_GRASPED";
static const std::string BOX_STATE_GRASPED = "GRASPED";
static const std::string BOX_STATE_RELEASED = "RELEASED";

// Load state
static const std::string LOAD_STATE_UNKNOWN = "UNKNOWN";
static const std::string LOAD_STATE_EMPTY = "EMPTY";
static const std::string LOAD_STATE_LOADED = "LOADED";

}  // namespace constants

}  // namespace cs625_task_manager

#endif  // CS625_TASK_MANAGER__TASK_DEFINITIONS_HPP_
