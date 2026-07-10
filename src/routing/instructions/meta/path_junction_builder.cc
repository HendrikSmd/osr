#include "osr/routing/instructions/meta/path_junction_builder.h"

namespace osr {

path_junction_builder::path_junction_builder(ways const& w) : builder_{w} {}

std::optional<visited_junction> path_junction_builder::build(
    path::segment const& arrival, path::segment const& departure) const {
  return builder_.build(
      arrival.from_, arrival.way_, arrival.to_, departure.way_, departure.to_,
      arrival.is_reverse_ ? direction::kBackward : direction::kForward,
      departure.is_reverse_ ? direction::kBackward : direction::kForward,
      arrival.mode_);
}

}  // namespace osr
