#include "osr/routing/instructions/instruction_util.h"

#include "osr/routing/instructions/module.h"

namespace osr {

bool is_from_motorway(ways const& w, visited_junction const& jctn) {
  const auto& segment_instruction_properties =
      w.way_instruction_properties_[jctn.get_arrive_way_idx()];

  return is_motorway_or_trunk(w, jctn.get_arrive_way_idx()) &&
         !segment_instruction_properties.is_link();
}

bool is_from_ramp(ways const& w, visited_junction const& jctn) {
  const auto& segment_instruction_properties =
      w.way_instruction_properties_[jctn.get_arrive_way_idx()];

  return is_motorway_or_trunk(w, jctn.get_arrive_way_idx()) &&
         segment_instruction_properties.is_link();
}

bool is_onto_motorway(ways const& w, visited_junction const& jctn) {
  const auto& exit_from_jctn = jctn.get_exit();

  const auto& segment_instruction_properties =
      w.way_instruction_properties_[exit_from_jctn.segment_way()];

  return is_motorway_or_trunk(w, exit_from_jctn.segment_way()) &&
         !segment_instruction_properties.is_link();
}

bool is_onto_ramp(ways const& w, visited_junction const& jctn) {
  const auto& exit_from_jctn = jctn.get_exit();

  const auto& segment_instruction_properties =
      w.way_instruction_properties_[exit_from_jctn.segment_way()];

  return is_motorway_or_trunk(w, exit_from_jctn.segment_way()) &&
         segment_instruction_properties.is_link();
}

bool is_last_segment(segment_contexts_window const& window) {
  return !window.has(1);
}

bool ways_have_same_name(way_idx_t const w1,
                         way_idx_t const w2,
                         ways const& w) {
  const string_idx_t s1 = w.way_names_[w1];
  if (s1 == string_idx_t::invalid()) {
    return false;
  }

  const string_idx_t s2 = w.way_names_[w2];
  if (s2 == string_idx_t::invalid()) {
    return false;
  }

  return s1 == s2;
}

bool ways_have_same_highway(way_idx_t const w1,
                            way_idx_t const w2,
                            ways const& w) {
  return w.way_instruction_properties_[w1].get_highway() ==
         w.way_instruction_properties_[w2].get_highway();
}

bool is_roundabout_way(ways const& w, way_idx_t const way) {
  if (way == way_idx_t::invalid()) {
    return false;
  }
  const auto junction = w.way_instruction_properties_[way].get_junction();
  return junction == junction::roundabout || junction == junction::circular;
}

bool is_link(ways const& w, way_idx_t const way) {
  return w.way_instruction_properties_[way].is_link();
}

bool is_motorway_or_trunk(ways const& w, way_idx_t const way) {
  const auto h = w.way_instruction_properties_[way].get_highway();
  return h == highway::motorway || h == highway::trunk;
}

bool turn_onto_through_street(ways const& w, visited_junction const& jctn) {

  const auto& exit = jctn.get_exit();
  const auto out_jctn_legs = jctn.get_outgoing_legs();

  for (std::size_t alt_idx = 1U; alt_idx < out_jctn_legs.size(); ++alt_idx) {
    if (jctn.is_exit_leg_idx(alt_idx)) {
      continue;
    }

    const auto& alt = out_jctn_legs[alt_idx];

    const bool is_nearly_straight = std::abs(alt.angle_with_exit()) > 160.0;
    const bool same_name =
        ways_have_same_name(alt.segment_way(), exit.segment_way(), w);
    const bool same_highway =
        ways_have_same_highway(alt.segment_way(), exit.segment_way(), w);

    if (is_nearly_straight && same_name && same_highway) {
      return true;
    }
  }

  return false;
}

bool is_more_important_by(highway const h1,
                          highway const h2,
                          unsigned const diff) {
  return h1 + diff <= h2;
}

bool is_segment_mode(path::segment const& segment, mode const m) {
  return segment.mode_ == m;
}

way_idx_t get_way_from(segment_context const& context) {
  return context.src_ptr_->way_;
}

std::pair<routing_instruction, routing_instruction> get_instruction_for_fork(
    ways const& w,
    visited_junction const& jctn,
    size_t const left,
    size_t const right) {
  utl::verify(left < right, "Outgoing legs are ordered in circular order");

  const auto out_jctn_legs = jctn.get_outgoing_legs();
  const auto& alt_left = out_jctn_legs[left];
  const auto& alt_right = out_jctn_legs[right];

  const auto alt_left_direction =
      get_relative_direction(alt_left.angle_with_arrive());
  const auto alt_right_direction =
      get_relative_direction(alt_right.angle_with_arrive());

  const auto& left_props =
      w.way_instruction_properties_[alt_left.segment_way()];
  const auto& right_props =
      w.way_instruction_properties_[alt_right.segment_way()];

  const auto left_highway = left_props.get_highway();
  const auto right_highway = right_props.get_highway();

  if (alt_left_direction == relative_direction::kStraight &&
      alt_right_direction != relative_direction::kStraight) {

    if (is_more_important_by(right_highway, left_highway, 1)) {
      return {stay_instruction{relative_direction::kLeft},
              stay_instruction{relative_direction::kRight}};
    }

    return {none_instruction{}, stay_instruction{relative_direction::kRight}};
  } else if (alt_right_direction == relative_direction::kStraight &&
             alt_left_direction != relative_direction::kStraight) {

    if (is_more_important_by(left_highway, right_highway, 1)) {
      return {stay_instruction{relative_direction::kLeft},
              stay_instruction{relative_direction::kRight}};
    }

    return {stay_instruction{relative_direction::kLeft}, none_instruction{}};
  }

  return {stay_instruction{relative_direction::kLeft},
          stay_instruction{relative_direction::kRight}};
}

std::tuple<routing_instruction, routing_instruction, routing_instruction>
get_instruction_for_fork(ways const& w,
                         visited_junction const& jctn,
                         size_t const left,
                         size_t const center,
                         size_t const right) {
  utl::verify(left < center && center < right,
              "Outgoing legs are ordered in circular order");

  const auto out_jctn_legs = jctn.get_outgoing_legs();
  const auto& alt_left = out_jctn_legs[left];
  const auto& alt_center = out_jctn_legs[center];
  const auto& alt_right = out_jctn_legs[right];

  if (alt_left.can_exit_hub() && alt_center.can_exit_hub() &&
      alt_right.can_exit_hub()) {

    const auto alt_left_direction =
        get_relative_direction(alt_left.angle_with_arrive());
    if (alt_left_direction == relative_direction::kStraight) {
      return {stay_instruction{relative_direction::kStraight},
              stay_instruction{relative_direction::kSlightRight},
              stay_instruction{relative_direction::kRight}};
    }

    const auto alt_right_direction =
        get_relative_direction(alt_right.angle_with_arrive());
    if (alt_right_direction == relative_direction::kStraight) {
      return {stay_instruction{relative_direction::kLeft},
              stay_instruction{relative_direction::kSlightLeft},
              stay_instruction{relative_direction::kStraight}};
    }

    // center is somewhat straight
    return {stay_instruction{relative_direction::kLeft},
            stay_instruction{relative_direction::kStraight},
            stay_instruction{relative_direction::kRight}};
  }

  if (alt_left.can_exit_hub()) {
    if (alt_right.can_exit_hub()) {
      const auto fork_instructions =
          get_instruction_for_fork(w, jctn, left, right);
      return {fork_instructions.first, none_instruction{},
              fork_instructions.second};
    }
    if (alt_center.can_exit_hub()) {
      const auto fork_instructions =
          get_instruction_for_fork(w, jctn, left, center);
      return {fork_instructions.first, fork_instructions.second,
              none_instruction{}};
    }

    return {stay_instruction{relative_direction::kLeft}, none_instruction{},
            none_instruction{}};
  }

  if (alt_right.can_exit_hub()) {
    if (alt_center.can_exit_hub()) {
      const auto fork_instructions =
          get_instruction_for_fork(w, jctn, center, right);

      return {none_instruction{}, fork_instructions.first,
              fork_instructions.second};
    }

    return {none_instruction{}, none_instruction{},
            stay_instruction{relative_direction::kRight}};
  }

  if (alt_center.can_exit_hub()) {
    return {none_instruction{}, stay_instruction{relative_direction::kStraight},
            none_instruction{}};
  }

  return {none_instruction{}, none_instruction{}, none_instruction{}};
}

unsigned long number_of_possible_turns_at(visited_junction const& hub,
                                          bool const left) {
  constexpr auto is_possible =
      [](visited_junction::junction_leg const& jctn_leg) {
        return jctn_leg.can_exit_hub();
      };

  const auto count =
      std::ranges::count_if(hub.get_exit_neighbors(left), is_possible);
  return static_cast<unsigned>(count);
}

std::optional<string_idx_t> get_crossing_street(ways const& w,
                                                visited_junction const& j) {
  const auto& exit_segment = j.get_exit();

  const auto from_way_idx = j.get_arrive_way_idx();
  const auto to_way_idx = exit_segment.segment_way();

  const auto& from_way_instruction_props =
      w.way_instruction_properties_[from_way_idx];
  const auto& to_way_instruction_props =
      w.way_instruction_properties_[to_way_idx];

  if (from_way_instruction_props.get_highway() != highway::footish ||
      to_way_instruction_props.get_highway() != highway::footish) {
    return std::nullopt;
  }

  const auto left_alts = j.get_exit_neighbors(true);
  const auto right_alts = j.get_exit_neighbors(false);

  string_idx_t crossed_street = string_idx_t::invalid();
  for (const auto& left_alt : left_alts) {
    const auto& left_way_instruction_props =
        w.way_instruction_properties_[left_alt.segment_way()];

    if (!is_more_important_by(left_way_instruction_props.get_highway(), footish,
                              1)) {
      continue;
    }

    for (const auto& right_alt : right_alts) {
      const auto& right_way_instruction_props =
          w.way_instruction_properties_[right_alt.segment_way()];

      if (!is_more_important_by(right_way_instruction_props.get_highway(),
                                footish, 1)) {
        continue;
      }

      if (!ways_have_same_name(left_alt.segment_way(), right_alt.segment_way(),
                               w)) {
        continue;
      }

      const auto street_found = w.way_names_[left_alt.segment_way()];
      if (crossed_street == string_idx_t::invalid()) {
        crossed_street = street_found;
      } else if (crossed_street != string_idx_t::invalid() &&
                 crossed_street != street_found) {
        // crossing multiple streets, cannot give definitive answer
        return std::nullopt;
      }
    }
  }
  if (crossed_street != string_idx_t::invalid()) {
    return std::optional{crossed_street};
  }

  return std::nullopt;
}

bool is_merge_on_through_street(ways const& w, visited_junction const& j) {
  const auto coming_from_way = j.get_arrive_way_idx();

  const auto exit = j.get_exit();
  const auto exit_way = exit.segment_way();
  if (ways_have_same_name(coming_from_way, exit_way, w)) {
    return false;
  }

  const bool is_left_merge = exit.angle_with_arrive() > 0;
  const auto jctn_out_legs = j.get_outgoing_legs();
  const auto get_opt_exit_predecessor =
      [&] -> std::optional<visited_junction::junction_leg> {
    const auto neighbor_idx_from = is_left_merge ? j.get_exit_leg_idx() + 1 : 1;
    const auto neighbor_idx_to =
        is_left_merge ? jctn_out_legs.size() : j.get_exit_leg_idx();
    for (auto neighbor_idx = neighbor_idx_from; neighbor_idx < neighbor_idx_to;
         neighbor_idx++) {
      const auto neighbor = jctn_out_legs[neighbor_idx];
      const auto neighbor_way = neighbor.segment_way();
      const auto angle_with_exit = j.angle_with_exit_from(w, neighbor_idx);
      if (ways_have_same_name(neighbor_way, exit_way, w) &&
          std::abs(angle_with_exit) < 20.0) {
        return std::make_optional(jctn_out_legs[neighbor_idx]);
      }
    }

    return std::nullopt;
  };
  const auto opt_exit_predecessor = get_opt_exit_predecessor();
  if (opt_exit_predecessor.has_value()) {
    return true;
  }
  return false;
}

bool is_pedestrian(mode const m) {
  return m == mode::kFoot || m == mode::kWheelchair;
}

}  // namespace osr
