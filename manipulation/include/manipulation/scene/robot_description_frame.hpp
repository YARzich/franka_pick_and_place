// Responsibility: Resolves and validates the fixed root frame encoded by robot_description.

#ifndef MANIPULATION__SCENE__ROBOT_DESCRIPTION_FRAME_HPP_
#define MANIPULATION__SCENE__ROBOT_DESCRIPTION_FRAME_HPP_

#include <string>

namespace manipulation::scene
{

/// Parsed fixed-root model-frame contract from one URDF document.
struct FixedRootFrameResult
{
  bool valid{false};
  std::string frame_id;
  std::string message;
};

/**
 * Parse the root link used as MoveIt's model frame in a fixed-root setup.
 *
 * This project deliberately has no SRDF virtual joint. For such robot models,
 * MoveIt's authoritative PlanningScene frame is the URDF root link. A backend
 * with a virtual joint must provide its own model-frame verifier instead of
 * using this fixed-root contract.
 */
FixedRootFrameResult parse_fixed_root_frame(const std::string & urdf_xml);

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__ROBOT_DESCRIPTION_FRAME_HPP_
