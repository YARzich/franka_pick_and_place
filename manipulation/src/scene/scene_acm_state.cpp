// Responsibility: Canonicalizes and applies AllowedCollisionMatrix updates in the scene state
//   model.

#include "scene_state_model_internal.hpp"

#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace manipulation::scene::detail
{

AcmState AcmState::from_message(
  const moveit_msgs::msg::AllowedCollisionMatrix & matrix)
{
  AcmState state;
  const auto size = matrix.entry_names.size();
  if (matrix.entry_values.size() != size) {
    throw std::runtime_error("allowed-collision matrix is not square");
  }
  std::set<std::string> names;
  for (const auto & name : matrix.entry_names) {
    if (name.empty() || !names.insert(name).second) {
      throw std::runtime_error("allowed-collision matrix contains an invalid entry name");
    }
  }
  if (matrix.default_entry_names.size() != matrix.default_entry_values.size()) {
    throw std::runtime_error("allowed-collision matrix defaults are inconsistent");
  }
  for (std::size_t index = 0; index < matrix.default_entry_names.size(); ++index) {
    const auto & name = matrix.default_entry_names[index];
    if (name.empty() || !state.defaults_.emplace(
        name, matrix.default_entry_values[index]).second)
    {
      throw std::runtime_error("allowed-collision matrix contains duplicate defaults");
    }
  }

  for (std::size_t first = 0; first < size; ++first) {
    if (matrix.entry_values[first].enabled.size() != size) {
      throw std::runtime_error("allowed-collision matrix is not square");
    }
    const auto diagonal_default = state.pair_default(
      matrix.entry_names[first], matrix.entry_names[first]);
    if (matrix.entry_values[first].enabled[first] != diagonal_default) {
      throw std::runtime_error(
              "ACM self entries cannot be preserved by MoveIt's message constructor");
    }
    for (std::size_t second = first + 1U; second < size; ++second) {
      const bool value = matrix.entry_values[first].enabled[second];
      if (matrix.entry_values[second].enabled.size() != size ||
        matrix.entry_values[second].enabled[first] != value)
      {
        throw std::runtime_error("allowed-collision matrix is not symmetric");
      }
      const auto key = ordered_pair(
        matrix.entry_names[first], matrix.entry_names[second]);
      if (value != state.pair_default(key.first, key.second)) {
        state.entries_[key] = value;
      }
    }
  }
  return state;
}

void AcmState::set_pair(
  const std::string & first,
  const std::string & second,
  const bool value)
{
  const auto key = ordered_pair(first, second);
  if (value == pair_default(key.first, key.second)) {
    entries_.erase(key);
  } else {
    entries_[key] = value;
  }
}

void AcmState::remove_entries(const std::string & name)
{
  for (auto iterator = entries_.begin(); iterator != entries_.end(); ) {
    if (iterator->first.first == name || iterator->first.second == name) {
      iterator = entries_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

moveit_msgs::msg::AllowedCollisionMatrix AcmState::to_message() const
{
  moveit_msgs::msg::AllowedCollisionMatrix matrix;
  std::set<std::string> names;
  for (const auto & [name, value] : defaults_) {
    (void)value;
    names.insert(name);
  }
  for (const auto & [pair, value] : entries_) {
    (void)value;
    names.insert(pair.first);
    names.insert(pair.second);
  }
  matrix.entry_names.assign(names.begin(), names.end());
  matrix.entry_values.resize(matrix.entry_names.size());
  for (std::size_t first = 0; first < matrix.entry_names.size(); ++first) {
    auto & row = matrix.entry_values[first].enabled;
    row.resize(matrix.entry_names.size(), false);
    for (std::size_t second = 0; second < matrix.entry_names.size(); ++second) {
      const auto key = ordered_pair(
        matrix.entry_names[first], matrix.entry_names[second]);
      const auto entry = entries_.find(key);
      row[second] = entry == entries_.end() ?
        pair_default(key.first, key.second) : entry->second;
    }
  }
  for (const auto & [name, value] : defaults_) {
    matrix.default_entry_names.push_back(name);
    matrix.default_entry_values.push_back(value);
  }
  return matrix;
}

std::pair<std::string, std::string> AcmState::ordered_pair(
  const std::string & first,
  const std::string & second)
{
  return first <= second ? std::make_pair(first, second) :
         std::make_pair(second, first);
}

bool AcmState::pair_default(const std::string & first, const std::string & second) const
{
  const auto first_default = defaults_.find(first);
  const auto second_default = defaults_.find(second);
  if (first_default == defaults_.end()) {
    return second_default == defaults_.end() ? false : second_default->second;
  }
  if (second_default == defaults_.end()) {
    return first_default->second;
  }
  return first_default->second && second_default->second;
}

}  // namespace manipulation::scene::detail
