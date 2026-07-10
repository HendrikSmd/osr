#pragma once

#include <span>
#include <vector>

#include "osr/routing/instructions/meta/way_segment.h"
#include "osr/routing/mode.h"
#include "osr/types.h"

namespace osr {

struct ways;
class visited_junction_builder;

class visited_junction {
public:
  struct junction_leg {
    junction_leg(double angle_with_exit,
                 double angle_with_arrive,
                 bool can_enter_hub,
                 bool can_exit_hub,
                 bool is_opposite_of_arrive,
                 way_segment segment);

    double angle_with_exit() const;
    double angle_with_arrive() const;
    bool can_enter_hub() const;
    bool can_exit_hub() const;
    bool is_opposite_of_arrive() const;
    way_segment const& segment() const;
    way_idx_t segment_way() const;

  private:
    double angle_with_exit_;
    double angle_with_arrive_;
    bool can_enter_hub_;
    bool can_exit_hub_;
    bool is_opposite_of_arrive_;
    way_segment segment_;
  };

  unsigned long number_of_possible_turns(bool consider_uturn) const;

  unsigned long number_of_visible_turns(bool consider_uturn) const;

  double angle_with_exit_from(ways const& w, std::size_t alt_idx) const;

  void print(std::ostream& out, ways const& w) const;

  junction_leg const& get_exit() const;

  way_segment get_arrive_way_segment() const;

  way_idx_t get_arrive_way_idx() const;

  bool is_exit_leg_idx(size_t i) const;

  size_t get_exit_leg_idx() const;

  std::span<const visited_junction::junction_leg> get_outgoing_legs() const;

  auto get_exit_neighbors(bool left) const {
    if (left) {
      // From index 1 (omit u-turn) to exit_from_hub_idx_ (exclusive)
      return std::ranges::subrange(
          outgoing_jctn_legs_.begin() + 1,
          outgoing_jctn_legs_.begin() + static_cast<long>(jctn_exit_idx_));
    } else {
      // From index exit_from_hub_idx_ + 1 to the end
      return std::ranges::subrange(
          outgoing_jctn_legs_.begin() + static_cast<long>(jctn_exit_idx_) + 1,
          outgoing_jctn_legs_.end());
    }
  }

  bool is_exit_natural_choice(ways const& w) const;

  bool is_end_of_turn_lane(ways const& w, bool left, bool dedicated) const;

  bool does_way_names_change(ways const& w) const;

  std::string to_json(ways const& w) const;

private:
  friend visited_junction_builder;

  visited_junction(way_segment arrive_way,
                   node_idx_t junction_hub,
                   std::vector<junction_leg>&& out_jct_legs,
                   size_t exit_idx);

  way_segment arrive_jctn_on_;
  node_idx_t jctn_hub_;
  std::vector<junction_leg> outgoing_jctn_legs_;
  size_t jctn_exit_idx_;
};

class visited_junction_builder {
public:
  visited_junction_builder(ways const& w);

  std::optional<visited_junction> build(node_idx_t const prev_node,
                                        way_idx_t const arrive_way,
                                        node_idx_t const hub_node,
                                        way_idx_t const depart_way,
                                        node_idx_t const next_node,
                                        direction const arrive_way_dir,
                                        direction const exit_way_dir,
                                        mode const m) const;

private:
  ways const& w_;
};

}  // namespace osr
