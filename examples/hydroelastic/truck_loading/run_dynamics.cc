#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(simulation_time, 0.4,
              "Desired duration of the simulation in seconds.");
// See MultibodyPlantConfig for the valid strings of contact_model.
DEFINE_string(discrete_solver, "tamsi",
              "Discrete contact solver. Options are: 'tamsi', 'sap'.");
DEFINE_string(contact_model, "hydroelastic",
              "Contact model. Options are: 'point', 'hydroelastic', "
              "'hydroelastic_with_fallback'.");
// See MultibodyPlantConfig for the valid strings of contact surface
// representation.
DEFINE_string(contact_surface_representation, "polygon",
              "Contact-surface representation for hydroelastics. "
              "Options are: 'triangle' or 'polygon'. Default is 'polygon'.");
DEFINE_double(hydroelastic_modulus, 3.0e4,
              "Hydroelastic modulus of the box, [Pa].");
DEFINE_double(resolution_hint_factor, 0.3,
              "This scaling factor, [unitless], multiplied by the radius of "
              "the box gives the target edge length of the mesh of the box "
              "on the surface of its hydroelastic representation. The smaller "
              "number gives a finer mesh with more tetrahedral elements.");
DEFINE_double(dissipation, 3.0,
              "Hunt & Crossley dissipation, [s/m], for the box");
DEFINE_double(friction_coefficient, 0.3,
              "coefficient for both static and dynamic friction, [unitless], "
              "of the box.");
DEFINE_double(mbp_dt, 0.001,
              "The fixed time step period (in seconds) of discrete updates "
              "for the multibody plant modeled as a discrete system. "
              "Strictly positive.");

DEFINE_double(slope_angle, 0.2, "slope roll angle in radian");

namespace drake {
namespace examples {
namespace bin_action {
namespace {

using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::CoulombFriction;
using drake::multibody::SpatialVelocity;
using Eigen::Vector3d;
using geometry::AddCompliantHydroelasticProperties;
using geometry::AddContactMaterial;
using geometry::Box;
using geometry::ProximityProperties;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::UnitInertia;

int do_main() {
  systems::DiagramBuilder<double> builder;

  multibody::MultibodyPlantConfig config;
  // We allow only discrete systems.
  DRAKE_DEMAND(FLAGS_mbp_dt > 0.0);
  config.time_step = FLAGS_mbp_dt;
  config.penetration_allowance = 0.001;
  config.contact_model = FLAGS_contact_model;
  config.discrete_contact_solver = FLAGS_discrete_solver;
  config.contact_surface_representation = FLAGS_contact_surface_representation;
  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
  RollPitchYawd(0.2, 0.0, 0.0),
      plant.mutable_gravity_field().set_gravity_vector(Vector3d{0, 0, -9.81});

  // load scene

  plant.Finalize();

  visualization::AddDefaultVisualization(&builder);

  auto diagram = builder.Build();
  auto simulator = MakeSimulatorFromGflags(*diagram);

  // Set the box's initial pose.
  systems::Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator->get_mutable_context());

  auto& box = plant.GetBodyByName(std::string("Box"));
  plant.SetFreeBodyPose(
      &plant_context, box,
      math::RigidTransformd{RollPitchYawd(FLAGS_slope_angle, 0.0, 0.0),
                            Vector3d(0.0, 0.0, 0.2)});

  simulator->AdvanceTo(FLAGS_simulation_time);
  systems::PrintSimulatorStatistics(*simulator);
  return 0;
}

}  // namespace
}  // namespace bin_action
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is an example of using the hydroelastic contact model with a\n"
      "non-convex collision geometry loaded from an SDFormat file of a\n"
      "dinner plate. The box, the plate, and the floor are compliant,\n"
      "rigid, and compliant hydroelastic. The plate-box, box-floor,\n"
      "and plate-floor contacts are rigid-compliant, compliant-compliant,\n"
      "and rigid-compliant. The hydroelastic contact model can work with\n"
      "non-convex shapes accurately without resorting to their convex\n"
      "hulls. Launch drake-visualizer before running this example.\n"
      "See the README.md file for more information.\n");
  FLAGS_simulator_publish_every_time_step = true;
  FLAGS_simulator_target_realtime_rate = 0.1;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::bin_action::do_main();
}
