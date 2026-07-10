#pragma once

#include "osr/routing/instructions/meta/visited_junction.h"
#include "osr/routing/path.h"

namespace osr {

class path_junction_builder {
public:
  explicit path_junction_builder(ways const& w);

  std::optional<visited_junction> build(path::segment const& arrival,
                                        path::segment const& departure) const;

private:
  visited_junction_builder builder_;
};

}  // namespace osr
