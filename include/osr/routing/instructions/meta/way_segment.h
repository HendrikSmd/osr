#pragma once

#include "osr/point.h"
#include "osr/types.h"

namespace osr {

struct ways;

class way_segment {

public:
  bool is_way_aligned() const;
  way_segment get_flipped() const;

  way_idx_t get_way_idx() const;

  osm_node_idx_t get_from_osm_idx(ways const& w) const;
  osm_node_idx_t get_to_osm_idx(ways const& w) const;

  std::uint16_t get_from_idx() const;
  std::uint16_t get_to_idx() const;

  point get_from_position(ways const& w) const;
  point get_to_position(ways const& w) const;

private:
  way_segment(way_idx_t way_idx, std::uint16_t from, std::uint16_t to);

  friend ways;

  way_idx_t way_idx_;
  std::uint16_t from_;
  std::uint16_t to_;  // inclusive
};

}  // namespace osr
