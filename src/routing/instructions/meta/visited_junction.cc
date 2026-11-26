#include "osr/routing/instructions/meta/visited_junction.h"

#include "boost/json.hpp"
#include <optional>

#include "osr/routing/instructions/instruction_util.h"
#include "osr/routing/instructions/meta/visited_junction.h"
#include "osr/types.h"
#include "utl/pipes/all.h"
#include "utl/pipes/transform.h"
#include "utl/pipes/vec.h"

namespace osr {

namespace {

distance_t get_min_exit_segment_distance_for_mode(ways const& w,
                                                  way_idx_t const way,
                                                  mode const m) {
  if (is_pedestrian(m)) {
    return 5;
  }
  const auto highway_type = w.way_instruction_properties_[way].get_highway();
  switch (highway_type) {
    case highway::motorway: return 100;
    case highway::trunk: return 80;
    case highway::primary:
    case highway::secondary: return 20;
    default: return 10;
  }
}

distance_t get_min_arrival_segment_distance_for_mode(ways const& w,
                                                     way_idx_t const way,
                                                     mode const m) {
  if (is_pedestrian(m)) {
    return 0;
  }
  const auto highway_type = w.way_instruction_properties_[way].get_highway();
  switch (highway_type) {
    case highway::motorway: return 30;
    case highway::trunk: return 20;
    case highway::primary:
    case highway::secondary: return 20;
    default: return 5;
  }
}

bool is_accessible(ways const& w, way_segment const& ws, mode const m) {
  const auto w_props = w.r_->way_properties_[ws.get_way_idx()];
  switch (m) {
    case mode::kBike:
      if (!w_props.is_bike_accessible()) return false;
      break;
    case mode::kCar:
    case mode::kHgv:
      if (!w_props.is_car_accessible()) return false;
      break;
    case mode::kFerry:
      if (!w_props.is_ferry_accessible()) return false;
      break;
    case mode::kRailway:
      if (!w_props.is_railway_accessible()) return false;
      break;
    case mode::kFoot:
    case mode::kWheelchair:
      if (!w_props.is_foot_accessible()) return false;
  }

  if (m == mode::kFoot || m == mode::kWheelchair) {
    return true;
  }

  const auto travel_dir =
      ws.is_way_aligned() ? direction::kForward : direction::kBackward;
  if (m == mode::kBike && w_props.is_oneway_bike() &&
      travel_dir == direction::kBackward) {
    return false;
  }
  if (m == mode::kCar && w_props.is_oneway_car() &&
      travel_dir == direction::kBackward) {
    return false;
  }

  return true;
}

void emplace_junction_leg(
    ways const& w,
    double const exit_angle,
    point const prev_hub,
    point const hub,
    way_segment const& ws,
    mode const m,
    bool const is_uturn,
    std::vector<visited_junction::junction_leg>& alternatives) {
  const auto& way_polyline = w.way_polylines_[ws.get_way_idx()];

  point const to = way_polyline[ws.get_to_idx()];
  double const alt_angle =
      get_angle(prev_hub.as_latlng(), hub.as_latlng(), to.as_latlng());
  double const diff = alt_angle - exit_angle;

  bool const can_exit_hub = is_accessible(w, ws, m);
  bool const can_enter_hub = is_accessible(w, ws.get_flipped(), m);
  double const relative_diff = normalize(diff);
  alternatives.emplace_back(relative_diff, alt_angle, can_enter_hub,
                            can_exit_hub, is_uturn, ws);
}

struct junction_geometry {
  point from_;
  point hub_;
  point to_;
};

junction_geometry get_junction_geometry(ways const& w,
                                        way_segment const& arrive,
                                        way_segment const& exit,
                                        mode const m) {
  const auto& arrive_polyline = w.way_polylines_[arrive.get_way_idx()];
  const auto& exit_polyline = w.way_polylines_[exit.get_way_idx()];

  const point hub = arrive.get_to_position(w);
  const auto arrive_prev_idx = w.get_next_osm_way_node_idx_or_last(
      arrive.get_way_idx(), arrive.get_to_idx(),
      arrive.is_way_aligned() ? direction::kBackward : direction::kForward,
      get_min_arrival_segment_distance_for_mode(w, arrive.get_way_idx(), m));
  const point from = arrive_polyline[arrive_prev_idx];

  const auto exit_next_idx = w.get_next_osm_way_node_idx_or_last(
      exit.get_way_idx(), exit.get_from_idx(),
      exit.is_way_aligned() ? direction::kForward : direction::kBackward,
      get_min_exit_segment_distance_for_mode(w, exit.get_way_idx(), m));
  const point to = exit_polyline[exit_next_idx];

  return {.from_ = from, .hub_ = hub, .to_ = to};
}

void add_outgoing_legs(ways const& w,
                       node_idx_t const hub_node,
                       way_segment const& arrive_jctn_on,
                       way_segment const& exit_jctn_with,
                       double const exit_angle,
                       point const from_point,
                       point const hub_point,
                       mode const m,
                       std::vector<visited_junction::junction_leg>& out_legs) {
  const auto& alt_ways = w.r_->node_ways_[hub_node];
  const auto osm_hub_node = w.node_to_osm_[hub_node];
  for (const auto alt_way : alt_ways) {

    // Forward Search
    const auto forward_seg =
        w.make_segment(alt_way, osm_hub_node, direction::kForward,
                       get_min_exit_segment_distance_for_mode(w, alt_way, m));

    if (forward_seg.has_value() && (alt_way != exit_jctn_with.get_way_idx() ||
                                    !exit_jctn_with.is_way_aligned())) {

      const bool is_uturn_alt =
          alt_way == arrive_jctn_on.get_way_idx() &&
          forward_seg->is_way_aligned() ^ arrive_jctn_on.is_way_aligned();

      emplace_junction_leg(w, exit_angle, from_point, hub_point,
                           forward_seg.value(), m, is_uturn_alt, out_legs);
    }

    // Backward search
    const auto backward_seg =
        w.make_segment(alt_way, osm_hub_node, direction::kBackward,
                       get_min_exit_segment_distance_for_mode(w, alt_way, m));

    if (backward_seg.has_value() && (alt_way != exit_jctn_with.get_way_idx() ||
                                     exit_jctn_with.is_way_aligned())) {

      const bool is_uturn_alt =
          alt_way == arrive_jctn_on.get_way_idx() &&
          backward_seg->is_way_aligned() ^ arrive_jctn_on.is_way_aligned();

      emplace_junction_leg(w, exit_angle, from_point, hub_point,
                           backward_seg.value(), m, is_uturn_alt, out_legs);
    }
  }
}

}  // namespace

visited_junction::junction_leg const& visited_junction::get_exit() const {
  return outgoing_jctn_legs_[jctn_exit_idx_];
}

way_idx_t visited_junction::get_arrive_way_idx() const {
  return arrive_jctn_on_.get_way_idx();
}

way_segment visited_junction::get_arrive_way_segment() const {
  return arrive_jctn_on_;
}

bool visited_junction::is_exit_leg_idx(size_t i) const {
  return i == jctn_exit_idx_;
}

size_t visited_junction::get_exit_leg_idx() const { return jctn_exit_idx_; }

std::span<const visited_junction::junction_leg>
visited_junction::get_outgoing_legs() const {
  return outgoing_jctn_legs_;
}

unsigned long visited_junction::number_of_possible_turns(
    bool const consider_uturn) const {
  constexpr auto is_possible =
      [](visited_junction::junction_leg const& rel_seg) {
        return rel_seg.can_exit_hub();
      };
  // drop because we omit the u-turn
  const auto count = std::ranges::count_if(
      outgoing_jctn_legs_ | std::views::drop(consider_uturn ? 0 : 1),
      is_possible);
  return static_cast<unsigned>(count);
}

unsigned long visited_junction::number_of_visible_turns(
    bool const consider_uturn) const {
  return static_cast<unsigned>(outgoing_jctn_legs_.size() -
                               (consider_uturn ? 0 : 1));
}

double visited_junction::angle_with_exit_from(ways const& w,
                                              std::size_t const alt_idx) const {
  utl::verify(alt_idx < outgoing_jctn_legs_.size(),
              "Alternative out of bounds");
  if (alt_idx == jctn_exit_idx_) {
    return 0.0;
  }

  const auto& from = outgoing_jctn_legs_[alt_idx];
  const auto& exit = get_exit();

  // This must be get_to_idx because the direction
  // of alternative way segment at a hub is always from
  // hub
  const auto from_node_p = from.segment().get_to_position(w);
  const auto hub_node_p = exit.segment().get_from_position(w);
  const auto exit_node_p = exit.segment().get_to_position(w);
  return get_angle(from_node_p.as_latlng(), hub_node_p.as_latlng(),
                   exit_node_p.as_latlng());
}

void visited_junction::print(std::ostream& out, ways const& w) const {
  out << "hub_node=" << w.node_to_osm_[jctn_hub_]
      << ", arrive_on=" << w.way_osm_idx_[arrive_jctn_on_.get_way_idx()];

  auto const print_alts =
      [&](std::string_view name,
          std::vector<visited_junction::junction_leg> const& alts) {
        out << ", " << name << "=[";
        for (auto i = 0U; i < alts.size(); ++i) {
          auto const& alt = alts[i];
          out << "{way=" << w.way_osm_idx_[alt.segment_way()]
              << ", angle_with_exit=" << alt.angle_with_exit()
              << ", angle_with_arrive=" << alt.angle_with_arrive()
              << ", can_enter_hub=" << (alt.can_enter_hub() ? "true" : "false")
              << ", can_exit_hub=" << (alt.can_exit_hub() ? "true" : "false")
              << ", is_exit_taken=" << (i == jctn_exit_idx_ ? "true" : "false")
              << "}";
          if (i != alts.size() - 1) {
            out << ", ";
          }
        }
        out << "]";
      };

  print_alts("alternatives", outgoing_jctn_legs_);
}

// TODO
bool visited_junction::is_exit_natural_choice(ways const& w) const {
  if (!ways_have_same_name(arrive_jctn_on_.get_way_idx(),
                           get_exit().segment_way(), w)) {
    return false;
  }

  return true;
}

bool visited_junction::is_end_of_turn_lane(ways const& w,
                                           bool left,
                                           bool const dedicated) const {
  constexpr auto is_dedicated_turn_lane =
      [](way_instruction_properties const& props, direction const dir,
         bool const l) -> bool {
    return (l ? props.has_left_turn_lane(dir) && !props.has_right_turn_lane(dir)
              : props.has_right_turn_lane(dir) &&
                    !props.has_left_turn_lane(dir)) &&
           !props.has_through_lane(dir);
  };

  constexpr auto is_turn_lane = [](way_instruction_properties const& props,
                                   direction const dir, bool const l) -> bool {
    return l ? props.has_left_turn_lane(dir) : props.has_right_turn_lane(dir);
  };

  const auto applies = dedicated ? is_dedicated_turn_lane : is_turn_lane;

  direction const arrive_dir = arrive_jctn_on_.is_way_aligned()
                                   ? direction::kForward
                                   : direction::kBackward;
  const auto arrive_props =
      w.way_instruction_properties_[arrive_jctn_on_.get_way_idx()];
  const auto is_arrive_dedicated_turn_lane =
      applies(arrive_props, arrive_dir, left);
  if (!is_arrive_dedicated_turn_lane) {
    return false;
  }

  const auto& exit = get_exit();
  const auto exit_props = w.way_instruction_properties_[exit.segment_way()];
  direction const exit_dir = exit.segment().is_way_aligned()
                                 ? direction::kForward
                                 : direction::kBackward;
  return !applies(exit_props, exit_dir, left);
}

bool visited_junction::does_way_names_change(ways const& w) const {
  const auto arrive_way = arrive_jctn_on_.get_way_idx();
  const auto exit_way = get_exit().segment_way();
  return !ways_have_same_name(arrive_way, exit_way, w);
}

std::string visited_junction::to_json(ways const& w) const {
  const auto way_segment_to_json = [&w](way_segment const& way_seg) {
    if (way_seg.get_way_idx() == way_idx_t::invalid()) {
      return boost::json::object{{
                                     "way_id",
                                     0U,
                                 },
                                 {"node_range", boost::json::array{0, 0}}};
    }
    const auto& way_osm_nodes = w.way_osm_nodes_[way_seg.get_way_idx()];
    return boost::json::object{
        {"way_id", to_idx(w.way_osm_idx_[way_seg.get_way_idx()])},
        {"node_range",
         boost::json::array{to_idx(way_osm_nodes[way_seg.get_from_idx()]),
                            to_idx(way_osm_nodes[way_seg.get_to_idx()])}}};
  };

  const auto relative_way_segment_to_json =
      [&way_segment_to_json](junction_leg const& rel_way_seg) {
        return boost::json::object{
            {"segment", way_segment_to_json(rel_way_seg.segment())},
            {"angle_with_exit", rel_way_seg.angle_with_exit()},
            {"angle_with_arrive", rel_way_seg.angle_with_arrive()},
            {"can_enter_hub", rel_way_seg.can_enter_hub()},
            {"can_exit_hub", rel_way_seg.can_exit_hub()},
            {"is_opposite_of_arrive", rel_way_seg.is_opposite_of_arrive()}};
      };

  const auto rel_segments_to_json_array =
      [&relative_way_segment_to_json](
          std::vector<visited_junction::junction_leg> const& rel_segments) {
        return utl::all(rel_segments) |
               utl::transform(
                   [&](visited_junction::junction_leg const& rel_way_seg) {
                     return relative_way_segment_to_json(rel_way_seg);
                   }) |
               utl::emplace_back_to<boost::json::array>();
      };

  return boost::json::serialize(boost::json::object{
      {"arrive_hub_on", way_segment_to_json(arrive_jctn_on_)},
      {"hub_node", jctn_hub_ == node_idx_t::invalid()
                       ? 0U
                       : to_idx(w.node_to_osm_[jctn_hub_])},
      {"alternatives", rel_segments_to_json_array(outgoing_jctn_legs_)},
      {"exit_from_hub_idx", jctn_exit_idx_}});
}

visited_junction::junction_leg::junction_leg(double const angle_with_exit,
                                             double const angle_with_arrive,
                                             bool const can_enter_hub,
                                             bool const can_exit_hub,
                                             bool const is_opposite_of_arrive,
                                             way_segment segment)
    : angle_with_exit_{angle_with_exit},
      angle_with_arrive_{angle_with_arrive},
      can_enter_hub_{can_enter_hub},
      can_exit_hub_{can_exit_hub},
      is_opposite_of_arrive_{is_opposite_of_arrive},
      segment_{std::move(segment)} {}

double visited_junction::junction_leg::angle_with_exit() const {
  return angle_with_exit_;
}

double visited_junction::junction_leg::angle_with_arrive() const {
  return angle_with_arrive_;
}

bool visited_junction::junction_leg::can_enter_hub() const {
  return can_enter_hub_;
}

bool visited_junction::junction_leg::can_exit_hub() const {
  return can_exit_hub_;
}

bool visited_junction::junction_leg::is_opposite_of_arrive() const {
  return is_opposite_of_arrive_;
}

way_segment const& visited_junction::junction_leg::segment() const {
  return segment_;
}

way_idx_t visited_junction::junction_leg::segment_way() const {
  return segment_.get_way_idx();
}

visited_junction::visited_junction(way_segment arrive_way,
                                   node_idx_t junction_hub,
                                   std::vector<junction_leg>&& out_jct_legs,
                                   size_t exit_idx)
    : arrive_jctn_on_{arrive_way},
      jctn_hub_{junction_hub},
      outgoing_jctn_legs_{std::move(out_jct_legs)},
      jctn_exit_idx_{exit_idx} {};

visited_junction_builder::visited_junction_builder(ways const& w) : w_{w} {};

std::optional<visited_junction> visited_junction_builder::build(
    node_idx_t const prev_node,
    way_idx_t const arrive_way,
    node_idx_t const hub_node,
    way_idx_t const depart_way,
    node_idx_t const next_node,
    direction const arrive_way_dir,
    direction const exit_way_dir,
    mode const m) const {

  if (arrive_way == way_idx_t::invalid() ||
      depart_way == way_idx_t::invalid() ||
      prev_node == node_idx_t::invalid() || hub_node == node_idx_t::invalid() ||
      next_node == node_idx_t::invalid()) {
    return std::nullopt;
  }

  osm_node_idx_t const osm_jctn_hub_node = w_.node_to_osm_[hub_node];
  osm_node_idx_t const osm_prev_jctn_node = w_.node_to_osm_[prev_node];
  osm_node_idx_t const osm_next_jctn_node = w_.node_to_osm_[next_node];

  const auto arrive_jctn_segment = w_.make_segment(
      arrive_way, osm_prev_jctn_node, osm_jctn_hub_node, arrive_way_dir);
  if (!arrive_jctn_segment.has_value()) {
    return std::nullopt;
  }
  const auto exit_jctn_segment = w_.make_segment(
      depart_way, osm_jctn_hub_node, osm_next_jctn_node, exit_way_dir);
  if (!exit_jctn_segment.has_value()) {
    return std::nullopt;
  }

  auto const geo = get_junction_geometry(w_, arrive_jctn_segment.value(),
                                         exit_jctn_segment.value(), m);
  const auto exit_angle = get_angle(geo.from_.as_latlng(), geo.hub_.as_latlng(),
                                    geo.to_.as_latlng());

  std::vector<visited_junction::junction_leg> out_legs;
  add_outgoing_legs(w_, hub_node, arrive_jctn_segment.value(),
                    exit_jctn_segment.value(), exit_angle, geo.from_, geo.hub_,
                    m, out_legs);

  const auto u_turn_seg =
      std::ranges::find_if(out_legs, [](const auto& rel_way_seg) {
        return rel_way_seg.is_opposite_of_arrive();
      });

  if (u_turn_seg == out_legs.end()) {
    return std::nullopt;
  }

  if (u_turn_seg != out_legs.begin()) {
    std::iter_swap(out_legs.begin(), u_turn_seg);
  }

  std::ranges::sort(out_legs | std::views::drop(1), std::ranges::greater{},
                    &visited_junction::junction_leg::angle_with_arrive);

  auto exit_segment = visited_junction::junction_leg{
      0.0,  exit_angle, is_accessible(w_, exit_jctn_segment->get_flipped(), m),
      true, false,      exit_jctn_segment.value()};

  auto iter = std::ranges::upper_bound(
      out_legs | std::views::drop(1), exit_segment.angle_with_arrive(),
      std::ranges::greater{},
      &visited_junction::junction_leg::angle_with_arrive);

  auto const inserted_iter = out_legs.insert(iter, exit_segment);
  const auto exit_from_hub_idx =
      static_cast<unsigned>(std::distance(out_legs.begin(), inserted_iter));

  return visited_junction(arrive_jctn_segment.value(), hub_node,
                          std::move(out_legs), exit_from_hub_idx);
}

}  // namespace osr
