// Responsibility: Parses URDF topology to identify a unique fixed robot root for PlanningScene use.

#include "manipulation/scene/robot_description_frame.hpp"

#include <urdf/model.h>

#include <string>

#include "manipulation/validation/frame_id.hpp"

namespace manipulation::scene
{

FixedRootFrameResult parse_fixed_root_frame(const std::string & urdf_xml)
{
  if (urdf_xml.empty()) {
    return {false, {}, "robot description is empty"};
  }

  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    return {false, {}, "robot description is not valid URDF"};
  }
  const auto root = model.getRoot();
  if (!root ||
    !manipulation::validation::is_canonical_relative_frame_id(root->name))
  {
    return {false, {}, "URDF root link is not a valid relative frame ID"};
  }
  return {true, root->name, "fixed-root URDF model frame parsed"};
}

}  // namespace manipulation::scene
