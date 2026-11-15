// Responsibility: Validates strategy identities and enforces the one-entry production registry.

#include "manipulation/workflow/strategy_registry.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace manipulation::workflow
{
namespace
{

bool valid_strategy_id(const std::string_view id)
{
  return !id.empty() && id.size() <= 128U &&
         std::all_of(id.cbegin(), id.cend(), [](const char character) {
             const auto byte = static_cast<unsigned char>(character);
             return byte >= 0x21U && byte <= 0x7eU && character != '|';
         });
}

std::map<std::string, std::unique_ptr<TransferExecutionStrategy>> make_registry(
  std::vector<std::unique_ptr<TransferExecutionStrategy>> strategies)
{
  if (strategies.empty()) {
    throw std::invalid_argument("strategy registry must not be empty");
  }
  std::map<std::string, std::unique_ptr<TransferExecutionStrategy>> result;
  for (auto & strategy : strategies) {
    if (!strategy) {
      throw std::invalid_argument("strategy registry received a null strategy");
    }
    const std::string id{strategy->id()};
    if (!valid_strategy_id(id)) {
      throw std::invalid_argument("strategy ID must be a bounded visible token");
    }
    const auto inserted = result.emplace(id, std::move(strategy));
    if (!inserted.second) {
      throw std::invalid_argument("strategy registry contains duplicate ID '" + id + "'");
    }
  }
  return result;
}

std::string unknown_strategy_message(
  const std::string_view requested,
  const std::map<std::string, std::unique_ptr<TransferExecutionStrategy>> & strategies)
{
  std::ostringstream message;
  message << "unknown transfer strategy '" << requested << "'; available:";
  for (const auto & entry : strategies) {
    message << ' ' << entry.first;
  }
  return message.str();
}

}  // namespace

StrategyRegistry StrategyRegistry::production(
  std::unique_ptr<TransferExecutionStrategy> pick_lift_place)
{
  if (!pick_lift_place || pick_lift_place->id() != kPickLiftPlaceStrategyId) {
    throw std::invalid_argument(
            "production strategy must report the exact ID 'pick_lift_place'");
  }
  std::vector<std::unique_ptr<TransferExecutionStrategy>> strategies;
  strategies.emplace_back(std::move(pick_lift_place));
  return StrategyRegistry(make_registry(std::move(strategies)));
}

StrategyRegistry StrategyRegistry::for_testing(
  std::vector<std::unique_ptr<TransferExecutionStrategy>> strategies)
{
  return StrategyRegistry(make_registry(std::move(strategies)));
}

StrategyRegistry::StrategyRegistry(
  std::map<std::string, std::unique_ptr<TransferExecutionStrategy>> strategies)
: strategies_(std::move(strategies))
{
}

TransferExecutionStrategy & StrategyRegistry::require(const std::string_view strategy_id)
{
  const auto found = strategies_.find(std::string{strategy_id});
  if (found == strategies_.end()) {
    throw std::invalid_argument(unknown_strategy_message(strategy_id, strategies_));
  }
  return *found->second;
}

const TransferExecutionStrategy & StrategyRegistry::require(
  const std::string_view strategy_id) const
{
  const auto found = strategies_.find(std::string{strategy_id});
  if (found == strategies_.cend()) {
    throw std::invalid_argument(unknown_strategy_message(strategy_id, strategies_));
  }
  return *found->second;
}

std::vector<std::string> StrategyRegistry::available_ids() const
{
  std::vector<std::string> result;
  result.reserve(strategies_.size());
  for (const auto & entry : strategies_) {
    result.emplace_back(entry.first);
  }
  return result;
}

std::size_t StrategyRegistry::size() const noexcept
{
  return strategies_.size();
}

}  // namespace manipulation::workflow
