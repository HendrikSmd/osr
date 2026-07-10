#include "osr/routing/instructions/meta/way_segment.h"
#include "osr/ways.h"

namespace osr {

way_segment::way_segment(way_idx_t way_idx,
                         std::uint16_t from,
                         std::uint16_t to)
    : way_idx_{way_idx}, from_{from}, to_{to} {};

bool way_segment::is_way_aligned() const { return from_ <= to_; }

way_segment way_segment::get_flipped() const {
  return way_segment(way_idx_, to_, from_);
}

way_idx_t way_segment::get_way_idx() const { return way_idx_; }

osm_node_idx_t way_segment::get_from_osm_idx(ways const& w) const {
  return w.way_osm_nodes_[way_idx_][from_];
}

osm_node_idx_t way_segment::get_to_osm_idx(ways const& w) const {
  return w.way_osm_nodes_[way_idx_][to_];
}

std::uint16_t way_segment::get_from_idx() const { return from_; }

std::uint16_t way_segment::get_to_idx() const { return to_; }

point way_segment::get_from_position(ways const& w) const {
  return w.way_polylines_[way_idx_][from_];
}

point way_segment::get_to_position(ways const& w) const {
  return w.way_polylines_[way_idx_][to_];
}

}  // namespace osr
