#include "osr/routing/instructions/modules/intersection_module.h"

#include <cmath>
#include <optional>

#include "osr/debug.h"
#include "osr/routing/instructions/directions.h"
#include "osr/routing/instructions/instruction_util.h"
#include "osr/routing/instructions/meta/visited_junction.h"
#include "osr/routing/instructions/routing_instruction.h"

namespace osr {

bool is_real_turn(relative_direction const dir) {
  return dir != relative_direction::kStraight &&
         dir != relative_direction::kSlightRight &&
         dir != relative_direction::kSlightLeft &&
         dir != relative_direction::kInvalid;
}

routing_instruction turn_action_from(relative_direction dir) {
  // Map relative direction to turn instruction
  switch (dir) {
    case relative_direction::kStraight: return continue_instruction{};
    case relative_direction::kSlightRight:
    case relative_direction::kRight:
    case relative_direction::kSharpRight:
    case relative_direction::kTurnAround:
    case relative_direction::kSharpLeft:
    case relative_direction::kLeft:
    case relative_direction::kSlightLeft: return turn_instruction{dir};
    case relative_direction::kInvalid: return none_instruction{};
  }
  return none_instruction{};
}

void set_segment_instruction(path::segment& seg,
                             routing_instruction const action) {
  seg.instruction_annotation_ = action;
}

bool is_simple_u_turn(visited_junction const& jctn) {
  const auto& arrive_on = jctn.get_arrive_way_segment();
  const auto& exit_on = jctn.get_exit().segment();

  return arrive_on.get_way_idx() == exit_on.get_way_idx() &&
         (arrive_on.is_way_aligned() ^ exit_on.is_way_aligned());
}

std::optional<visited_junction::junction_leg> get_next_alternative(
    visited_junction const& jctn, bool const to_left) {
  const auto neighbors = jctn.get_exit_neighbors(to_left);
  if (neighbors.empty()) {
    return std::nullopt;
  }

  return to_left ? neighbors.back() : neighbors.front();
}

std::optional<visited_junction::junction_leg> get_other_continue(
    visited_junction const& jctn, double const cutoff_angle) {
  std::optional<visited_junction::junction_leg> best;

  auto const consider = [&](visited_junction::junction_leg const& alt) {
    if (!alt.can_exit_hub() || alt.is_opposite_of_arrive()) {
      return;
    }
    double const abs_angle = std::abs(alt.angle_with_arrive());
    if (abs_angle > std::abs(cutoff_angle)) {
      return;
    }

    if (!best.has_value() || abs_angle < std::abs(best->angle_with_arrive())) {
      best = alt;
    }
  };

  for (auto const& alt : jctn.get_exit_neighbors(true)) {
    consider(alt);
  }
  for (auto const& alt : jctn.get_exit_neighbors(false)) {
    consider(alt);
  }

  return best;
}

bool outgoing_edges_are_slower_by_factor(ways const& w,
                                         visited_junction const& jctn,
                                         double const factor) {
  auto const chosen_speed =
      static_cast<double>(w.r_->way_properties_[jctn.get_exit().segment_way()]
                              .max_speed_km_per_h());
  if (chosen_speed <= 0.0) {
    return false;
  }

  auto const is_relevant = [](visited_junction::junction_leg const& alt) {
    return alt.can_exit_hub() && !alt.is_opposite_of_arrive();
  };

  for (auto const& alt : jctn.get_exit_neighbors(true)) {
    if (!is_relevant(alt)) {
      continue;
    }
    auto const alt_speed = static_cast<double>(
        w.r_->way_properties_[alt.segment_way()].max_speed_km_per_h());
    if (alt_speed >= chosen_speed / factor) {
      return false;
    }
  }

  for (auto const& alt : jctn.get_exit_neighbors(false)) {
    if (!is_relevant(alt)) {
      continue;
    }
    auto const alt_speed = static_cast<double>(
        w.r_->way_properties_[alt.segment_way()].max_speed_km_per_h());
    if (alt_speed >= chosen_speed / factor) {
      return false;
    }
  }

  return true;
}

bool same_road_signature(ways const& w, way_idx_t const a, way_idx_t const b) {
  if (a == way_idx_t::invalid() || b == way_idx_t::invalid()) {
    return false;
  }

  if (!ways_have_same_name(a, b, w)) {
    return false;
  }

  auto const pa = w.way_instruction_properties_[a];
  auto const pb = w.way_instruction_properties_[b];
  return pa.get_highway() == pb.get_highway() && pa.is_link() == pb.is_link();
}

bool is_major_road(way_instruction_properties const props) {
  auto const h = props.get_highway();
  return h == motorway || h == trunk || h == primary || h == secondary ||
         h == tertiary;
}

routing_instruction handle_real_turn(ways const& w,
                                     segment_context const& seg_context,
                                     segment_context const& next_seg_context) {
  const auto& jctn = seg_context.visited_junction_.value();
  const relative_direction next_seg_rel_dir =
      get_relative_direction(jctn.get_exit().angle_with_arrive());
  trace("┃ ║ handling real turn (exit_angle={})\n",
        jctn.get_exit().angle_with_arrive());
  utl::verify(is_real_turn(next_seg_rel_dir), "Not a real turn");

  auto& segment = *seg_context.src_ptr_;
  const auto& next_segment = *next_seg_context.src_ptr_;

  const routing_instruction potential_turn_type =
      turn_action_from(next_seg_rel_dir);

  trace("┃ ║ trying to check if a turn instruction is not needed\n");

  const auto num_possible_turns = jctn.number_of_possible_turns(false);
  const auto num_visible_turns = jctn.number_of_visible_turns(false);
  if (num_visible_turns <= 1) {
    trace("┃ ║ number of visible turns ≤ 1 -> no instruction needed!\n");
    return none_instruction{};
  }

  const bool stay_on_same_name =
      ways_have_same_name(segment.way_, next_segment.way_, w);

  if (num_possible_turns <= 1) {
    if (stay_on_same_name) {
      trace(
          "┃ ║ number of possible turns ≤ 1 and we continue on same street "
          "-> "
          "no instruction needed!\n");
      return none_instruction{};
    } else if (!jctn.is_end_of_turn_lane(w, true, true) &&
               !jctn.is_end_of_turn_lane(w, false, true)) {
      trace(
          "┃ ║ number of possible turns ≤ 1 and do not continue on same street "
          "-> "
          "kContinue is sufficient!\n");
      return continue_instruction{};
    }
  }

  // There are viable alternatives when arriving
  // at the hub (num_possible_turns > 1)
  if (stay_on_same_name && outgoing_edges_are_slower_by_factor(w, jctn, 2.0)) {
    trace(
        "┃ ║ number of possible turns > 1, continuing on same "
        "street, alternatives are less important\n");
    return none_instruction{};
  }

  trace("┃ ║ turn requires instruction\n");
  return potential_turn_type;
}

routing_instruction handle_slight_turn(
    ways const& w,
    segment_context const& seg_context,
    segment_context const& next_seg_context) {
  const auto& jctn = seg_context.visited_junction_.value();
  trace("┃ ║ handling slight turn (exit_angle={})\n",
        jctn.get_exit().angle_with_arrive());

  auto& segment = *seg_context.src_ptr_;
  const auto& next_segment = *next_seg_context.src_ptr_;

  const auto next_seg_rel_dir =
      get_relative_direction(jctn.get_exit().angle_with_arrive());
  const routing_instruction potential_turn_type =
      turn_action_from(next_seg_rel_dir);

  trace("┃ ║ trying to check if a turn instruction is not needed\n");
  trace("┃ ║ checking alternatives for a possible more natural continuation\n");

  constexpr auto continuation_abs_cutoff_angle = 50.;
  auto const other_continue =
      get_other_continue(jctn, continuation_abs_cutoff_angle);
  auto const leaving_current_street =
      !ways_have_same_name(segment.way_, next_segment.way_, w);

  if (other_continue.has_value()) {
    trace(
        "┃ ║ found the best alternative continuation in way = {} with degree "
        "= {}\n",
        other_continue->segment_way(), other_continue->angle_with_arrive());

    auto const delta = jctn.get_exit().angle_with_arrive();
    auto const other_delta = other_continue->angle_with_arrive();

    if (std::abs(delta) < std::abs(other_delta) && !leaving_current_street) {
      trace(
          "┃ ║ the exit is a more natural continuation -> no instruction "
          "required!\n");
      return none_instruction{};
    }

    if (ways_have_same_name(other_continue->segment_way(), segment.way_, w)) {
      trace("┃ ║ found a more natural continuation\n");
      trace("┃ ║ an instruction is needed\n");

      const bool left_of_other_continuation = other_delta < delta;
      trace("┃ ║ exit is to the {} of other continuation\n",
            left_of_other_continuation ? "left" : "right");
      trace("┃ ║ we check if there is another alternative further {}\n",
            left_of_other_continuation ? "left" : "right");

      const auto next_alternative =
          get_next_alternative(jctn, left_of_other_continuation);

      bool relevant_alternative_found =
          next_alternative.has_value() &&
          !next_alternative->is_opposite_of_arrive();
      if (relevant_alternative_found) {

        const auto next_segment_instruction_props =
            w.way_instruction_properties_[next_segment.way_];

        const auto alternative_instruction_props =
            w.way_instruction_properties_[next_alternative->segment()
                                              .get_way_idx()];

        if (is_more_important_by(next_segment_instruction_props.get_highway(),
                                 alternative_instruction_props.get_highway(),
                                 2)) {
          relevant_alternative_found = false;
        }
      }

      if (!relevant_alternative_found &&
          (!jctn.does_way_names_change(w) ||
           jctn.is_end_of_turn_lane(w, left_of_other_continuation, false))) {
        trace(
            "┃ ║ no other relevant possible turn on the {} and same street -> "
            "reside with "
            "stay {} "
            "instruction\n",
            left_of_other_continuation ? "left" : "right",
            left_of_other_continuation ? "left" : "right");
        return left_of_other_continuation
                   ? routing_instruction{stay_instruction{
                         relative_direction::kLeft}}
                   : routing_instruction{
                         stay_instruction{relative_direction::kRight}};
      }
      trace(
          "┃ ║ there is another relevant alternative further {} or street "
          "change\n",
          left_of_other_continuation ? "left" : "right");
    }
  } else {
    trace("┃ ║ found no continuation that is more natural than the exit\n");
    trace("┃ ║ check next if this is a merge on through street\n");
    if (is_merge_on_through_street(w, jctn)) {
      trace("┃ ║ is merge on through street\n");
      if (jctn.is_end_of_turn_lane(w, true, true)) {
        trace(
            "┃ ║ is merge on through street from dedicated left turn lane -> "
            "needs turn annotation\n");
        return turn_instruction{relative_direction::kLeft};
      } else if (jctn.is_end_of_turn_lane(w, false, true)) {
        trace(
            "┃ ║ is merge on through street from dedicated right turn lane -> "
            "needs turn annotation\n");
        return turn_instruction{relative_direction::kRight};
      }
    }

    return none_instruction{};
  }

  trace("┃ ║ turn requires turn instruction\n");
  return potential_turn_type;
}

void intersection_module::process(ways const& w,
                                  path&,
                                  segment_contexts_window const& window) {
  trace("┃ ╔ INTERSECTION MODULE\n");
  const auto& segment_context = *window.focus();
  const auto& next_segment_context = *window[1];

  auto& segment = *segment_context.src_ptr_;
  const auto& next_segment = *next_segment_context.src_ptr_;

  trace("┃ ║  -> looking at the next two segments\n");
  trace("┃ ║ Analysing <{}> -- way: {} -> <{}> -- way: {} -> <{}>\n",
        segment.from_, segment.way_, segment.to_, next_segment.way_,
        next_segment.to_);

  const auto& common_node_hub = segment_context.visited_junction_.value();

  if (is_simple_u_turn(common_node_hub)) {
    trace("┃ ║ Detected simple u-turn\n");
    trace("┃ ╚ annotation = kTurnAround\n");
    set_segment_instruction(segment,
                            turn_instruction{relative_direction::kTurnAround});
    return;
  }

  const relative_direction next_seg_rel_dir =
      get_relative_direction(common_node_hub.get_exit().angle_with_arrive());
  if (is_real_turn(next_seg_rel_dir)) {
    const auto instruction =
        handle_real_turn(w, segment_context, next_segment_context);
    trace("┃ ╚ annotation = {}\n", to_string(instruction));
    set_segment_instruction(segment, instruction);
  } else if (is_slightish(next_seg_rel_dir)) {
    const auto instruction =
        handle_slight_turn(w, segment_context, next_segment_context);
    trace("┃ ╚ annotation = {}\n", to_string(instruction));
    set_segment_instruction(segment, instruction);
  }
}

void intersection_module::post_process(
    ways const& w, path&, segment_contexts const& segment_contexts) {
  constexpr distance_t max_u_turn_bridge = 15U;

  for (auto ctx_it = segment_contexts.begin(); ctx_it != segment_contexts.end();
       ++ctx_it) {
    if (std::next(ctx_it) == segment_contexts.end()) {
      break;
    }

    const auto& seg_ctx = *ctx_it;
    const auto& next_seg_ctx = *std::next(ctx_it);
    if (!seg_ctx.visited_junction_.has_value() ||
        !next_seg_ctx.visited_junction_.has_value()) {
      continue;
    }

    auto& seg = *seg_ctx.src_ptr_;
    auto& next_seg = *next_seg_ctx.src_ptr_;
    if (std::holds_alternative<turn_instruction>(seg.instruction_annotation_) &&
        std::holds_alternative<turn_instruction>(
            next_seg.instruction_annotation_) &&
        next_seg.dist_ <= max_u_turn_bridge) {
      const auto& first_hub = seg_ctx.visited_junction_.value();
      const auto& second_hub = next_seg_ctx.visited_junction_.value();

      const auto combined_degree =
          std::abs(first_hub.get_exit().angle_with_arrive() +
                   second_hub.get_exit().angle_with_arrive());

      if (combined_degree < 170) {
        continue;
      }

      if (!ways_have_same_name(first_hub.get_arrive_way_idx(),
                               second_hub.get_exit().segment_way(), w)) {
        continue;
      }

      seg.instruction_annotation_ =
          turn_instruction{relative_direction::kTurnAround};
      next_seg.instruction_annotation_ = none_instruction{};
    }
  }
}

}  // namespace osr
