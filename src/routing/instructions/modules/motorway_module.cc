#include "osr/routing/instructions/modules/motorway_module.h"

#include <cmath>

#include "osr/debug.h"
#include "osr/routing/instructions/instruction_util.h"
#include "osr/routing/instructions/routing_instruction.h"

namespace osr {

bool handle_from_motorway(ways const& w,
                          visited_junction const& jctn,
                          path::segment& seg) {
  const auto num_visible_turns = jctn.number_of_visible_turns(true);

  const auto get_continue_index = [&](visited_junction const& j) {
    const auto out_jctn_legs = j.get_outgoing_legs();
    for (auto i = 1U; i < out_jctn_legs.size(); ++i) {
      const auto& alt = out_jctn_legs[i];

      if (!alt.can_exit_hub()) {
        continue;
      }

      const auto arrive_way = j.get_arrive_way_idx();
      const auto alt_way = alt.segment_way();

      const bool is_same_name = ways_have_same_name(arrive_way, alt_way, w);
      const bool is_same_highway =
          ways_have_same_highway(arrive_way, alt_way, w);

      if (is_same_name && is_same_highway && !is_link(w, alt_way)) {
        return static_cast<int>(i);
      }
    }

    return -1;
  };

  const auto get_most_likely_continue_index = [&](visited_junction const& j) {
    double best_abs_angle = 180.0;
    auto best_index = -1;
    const auto out_jctn_legs = j.get_outgoing_legs();
    for (auto i = 1U; i < out_jctn_legs.size(); ++i) {
      const auto& alt = out_jctn_legs[i];
      const auto alt_way = alt.segment_way();

      if (!alt.can_exit_hub() || is_link(w, alt_way) ||
          !is_motorway_or_trunk(w, alt_way)) {
        continue;
      }

      const auto abs_angle = std::abs(alt.angle_with_arrive());
      if (best_index < 0 || abs_angle < best_abs_angle) {
        best_abs_angle = abs_angle;
        best_index = static_cast<int>(i);
      }
    }

    return best_index;
  };

  const auto get_best_continue_index = [&](visited_junction const& j) {
    if (const auto continue_index = get_continue_index(j);
        continue_index >= 0) {
      return continue_index;
    }

    return get_most_likely_continue_index(j);
  };

  const auto best_continue_index = get_best_continue_index(jctn);
  const auto out_jctn_legs = jctn.get_outgoing_legs();
  if (best_continue_index < 0) {

    if (num_visible_turns == 2U) {
      // Ramp at the end of the highway
      seg.instruction_annotation_ = none_instruction{};
      return true;
    } else if (num_visible_turns == 3U) {
      if (out_jctn_legs[1].can_exit_hub() && out_jctn_legs[2].can_exit_hub()) {
        // splitting Ramp at the end of highway
        const auto fork_instructions = get_instruction_for_fork(w, jctn, 1, 2);
        seg.instruction_annotation_ = jctn.is_exit_leg_idx(1)
                                          ? fork_instructions.first
                                          : fork_instructions.second;
      } else {
        seg.instruction_annotation_ = none_instruction{};
      }
      return true;
    } else if (num_visible_turns == 4U) {
      // triple  fork at the end of highway
      const auto fork_instruction = get_instruction_for_fork(w, jctn, 1, 2, 3);
      seg.instruction_annotation_ =
          jctn.is_exit_leg_idx(1)   ? std::get<0>(fork_instruction)
          : jctn.is_exit_leg_idx(2) ? std::get<1>(fork_instruction)
                                    : std::get<2>(fork_instruction);
      return true;
    }

    return false;
  }

  const auto& exit = jctn.get_exit();
  if (is_link(w, exit.segment_way())) {
    if (jctn.get_exit_leg_idx() > static_cast<unsigned>(best_continue_index)) {
      seg.instruction_annotation_ =
          ramp_off_instruction{relative_direction::kRight};
    } else {  // hub.exit_from_hub_idx_ < best_continue_index
      seg.instruction_annotation_ =
          ramp_off_instruction{relative_direction::kLeft};
    }
    return true;
  }

  const auto n_exiting_motorways =  // >= 1 because continuation exists
      std::ranges::count_if(out_jctn_legs, [&](const auto& l) {
        const auto way = l.segment_way();
        return l.can_exit_hub() && is_motorway_or_trunk(w, way) &&
               !is_link(w, way);
      });

  if (n_exiting_motorways > 1 && is_motorway_or_trunk(w, exit.segment_way())) {

    std::vector<size_t> exiting_motorways;

    for (size_t i = 1U; i < out_jctn_legs.size(); ++i) {
      const auto& candidate = out_jctn_legs[i];
      const auto candidate_way = candidate.segment_way();
      if (candidate.can_exit_hub() && is_motorway_or_trunk(w, candidate_way) &&
          !is_link(w, candidate_way)) {
        exiting_motorways.push_back(i);
      }
    }

    // exit must be in exiting_motorways
    if (exiting_motorways.size() == 2U) {
      const auto fork_instructions = get_instruction_for_fork(
          w, jctn, exiting_motorways[0], exiting_motorways[1]);

      if (exiting_motorways[0] == jctn.get_exit_leg_idx()) {
        seg.instruction_annotation_ = fork_instructions.first;
      } else if (exiting_motorways[1] == jctn.get_exit_leg_idx()) {
        seg.instruction_annotation_ = fork_instructions.second;
      }
    } else if (exiting_motorways.size() == 3U) {
      const auto [il, ic, ir] =
          get_instruction_for_fork(w, jctn, exiting_motorways[0],
                                   exiting_motorways[1], exiting_motorways[2]);
      if (exiting_motorways[0] == jctn.get_exit_leg_idx()) {
        seg.instruction_annotation_ = il;
      } else if (exiting_motorways[1] == jctn.get_exit_leg_idx()) {
        seg.instruction_annotation_ = ic;
      } else if (exiting_motorways[2] == jctn.get_exit_leg_idx()) {
        seg.instruction_annotation_ = ir;
      }
    }

    return true;
  }

  return false;
}

bool handle_from_ramp(ways const& w,
                      visited_junction const& j,
                      path::segment& seg) {
  const auto num_possible_turns = j.number_of_possible_turns(true);
  const auto num_visible_turns = j.number_of_visible_turns(true);

  // ramp straight into a motorway/ramp
  if (num_visible_turns == 2U && num_possible_turns == 1U) {
    seg.instruction_annotation_ = none_instruction{};
    return true;
  } else if (num_visible_turns == 3U) {

    if (num_possible_turns == 1U && !is_onto_ramp(w, j)) {
      // merging onto a passing highway
      if (j.get_exit_leg_idx() == 2U) {
        // exit is second clockwise -> Merge left
        seg.instruction_annotation_ =
            ramp_on_instruction{relative_direction::kLeft};
      } else {
        // merge right
        seg.instruction_annotation_ =
            ramp_on_instruction{relative_direction::kRight};
      }
      return true;
    } else if (num_possible_turns == 2U) {
      // num_possible_turns == 2
      // -> Ramp fork
      if (j.get_exit_leg_idx() == 1U) {
        seg.instruction_annotation_ =
            stay_instruction{relative_direction::kLeft};
      } else {
        // h.exit_from_hub_idx_ == 2U
        seg.instruction_annotation_ =
            stay_instruction{relative_direction::kRight};
      }
      return true;
    }
  }

  return false;
}

void motorway_module::process(ways const& w,
                              path& p,
                              segment_contexts_window const& window) {
  trace("┃ ╔ MOTORWAY MODULE\n");
  if (is_last_segment(window)) {
    trace("┃ ║ is last segment, passing to next module\n");
    instruction_module::process(w, p, window);
    return;
  }

  const auto& segment_context = *window.focus();
  if (!segment_context.visited_junction_.has_value()) {
    trace("┃ ║ no hub, cannot annotate, passing to next module\n");
    instruction_module::process(w, p, window);
    return;
  }

  const auto& jctn = segment_context.visited_junction_.value();
  bool annotated = false;
  if (is_from_motorway(w, jctn) &&
      (is_onto_motorway(w, jctn) || is_onto_ramp(w, jctn))) {
    annotated = handle_from_motorway(w, jctn, *segment_context.src_ptr_);
  }
  if (is_from_ramp(w, jctn) &&
      (is_onto_motorway(w, jctn) || is_onto_ramp(w, jctn))) {
    annotated = handle_from_ramp(w, jctn, *segment_context.src_ptr_);
  }

  if (annotated) {
    return;
  }

  trace("┃ ╚ not responsible, passing to next module\n");
  instruction_module::process(w, p, window);
}

}  // namespace osr
