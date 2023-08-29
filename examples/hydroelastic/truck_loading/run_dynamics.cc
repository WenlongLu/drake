#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "nlohmann/json.hpp"
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
  multibody::Parser parser(&plant, &scene_graph);
  parser.AddModelsFromUrl(
      "package://drake/examples/hydroelastic/truck_loading/"
      "truck.sdf");

  auto pile_model_instance_index = plant.AddModelInstance("Pile");
  std::ifstream f("examples/hydroelastic/truck_loading/bin_state_test.json");
  nlohmann::json data = nlohmann::json::parse(f);
  nlohmann::json binState = data["binState"];
  nlohmann::json placedBoxes = binState["placedBoxes"];
  // std::string data_as_string = placedBoxes[0].dump();
  // std::cout << data_as_string << std::endl;
  for (long unsigned int i = 0; i < placedBoxes.size(); i++) {
    nlohmann::json jbox = placedBoxes[i];
    nlohmann::json jsize = jbox["size"];
    double box_size[] = {
        jsize["x"].template get<double>(),
        jsize["y"].template get<double>(),
        jsize["z"].template get<double>(),
    };
    const RigidBody<double>& box = plant.AddRigidBody(
        std::string("Box") + std::to_string(i), pile_model_instance_index,
        SpatialInertia<double>::SolidBoxWithMass(1.0, box_size[0], box_size[1],
                                                 box_size[2]));
    // Set up mechanical properties of the box.
    ProximityProperties box_props;
    AddContactMaterial(3.0, {} /* point stiffness */,
                       CoulombFriction<double>{0.3, 0.3}, &box_props);
    AddCompliantHydroelasticProperties(box_size[2] * 0.3, 3.0e4, &box_props);
    plant.RegisterCollisionGeometry(box, RigidTransformd::Identity(),
                                    Box(box_size[0], box_size[1], box_size[2]),
                                    "collision", std::move(box_props));
    // hybrid
    static double color_map[] = {
        1.23089864, 10.16553,  6.72044,  7.19493,  4.900085,  11.54753,
        8.646835,   4.26566,   13.92481, 14.01972, 5.401435,  3.09299,
        3.509375,   1.2726412, 4.88735,  60.05036, 12.554645, 6.16128,
        1.3000112,  6.107565,  57.87817, 6.68337,  7.138925,  4.91056,
        9.890295,   1.9495424, 6.019035, 4.781825, 5.03116,   324.82769,
        5.617405,   6.70499,   1.735371, 4.34455,  172.08611, 174.42302,
        3.82799,    3.84265,   1.814403, 1.823265};
    // linear
    // static double color_map[] = {
    //    2.45037, 10.5742, 7.93534, 8.78692, 5.93303, 10.5521, 7.87743, 3.76454,
    //    8.72636, 19.2609, 5.93145, 3.9343,  5.01203, 2.51871, 5.91926,
    //    108.509, 8.07319, 8.09182, 2.56059, 6.23021,
    //    109.149, 8.62658, 6.85653, 4.7726,
    //    9.01707, 3.82578, 5.91061, 5.78811, 5.37894, 644.67,  7.05693, 8.2915,
    //    2.2653,  4.81474, 340.23, 342.769, 5.22382, 6.81921, 2.4579,  2.4077};
    //  angular
    // static double color_map[] = {
    //    0.01142728, 9.75686,  5.50554, 5.60294,  3.86714,   12.54296, 9.41624,
    //    4.76678,    19.12326, 8.77854, 4.87142,  2.25168,   2.00672,
    //    0.0265724, 3.85544,    11.59172, 17.0361, 4.23074, 0.0394324, 5.98492,
    //    6.60734, 4.74016,    7.42132,  5.04852, 10.76352,
    //    0.0733048, 6.12746,  3.77554,
    //    4.68338,    4.98538,  4.17788, 5.11848,  1.205442,  3.87436,  3.94222,
    //    6.07704,    2.43216,  0.86609, 1.170906, 1.23883};
    double ratio = std::min(color_map[i], 10.0) / 10.0;
    const Vector4<double> orange(1.0 * (1 - ratio), 0.0, 1.0 * ratio, 1.0);
    plant.RegisterVisualGeometry(box, RigidTransformd::Identity(),
                                 Box(box_size[0], box_size[1], box_size[2]),
                                 "visual", orange);
  }

  plant.Finalize();

  visualization::AddDefaultVisualization(&builder);

  auto diagram = builder.Build();
  auto simulator = MakeSimulatorFromGflags(*diagram);

  // Set the box's initial pose.
  systems::Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator->get_mutable_context());

  for (long unsigned int i = 0; i < placedBoxes.size(); i++) {
    nlohmann::json jbox = placedBoxes[i];
    nlohmann::json jtranslation = jbox["translation"];
    double box_translation[] = {
        jtranslation["x"].template get<double>(),
        jtranslation["y"].template get<double>(),
        jtranslation["z"].template get<double>(),
    };
    nlohmann::json jrotation = jbox["rotation"];
    double box_rotation[] = {
        jrotation.contains("w") ? jrotation["w"].template get<double>() : 0.0,
        jrotation.contains("x") ? jrotation["x"].template get<double>() : 0.0,
        jrotation.contains("y") ? jrotation["y"].template get<double>() : 0.0,
        jrotation.contains("z") ? jrotation["z"].template get<double>() : 0.0,
    };
    // double box_rotation[] = {1, 0, 0, 0};
    auto& box = plant.GetBodyByName(std::string("Box") + std::to_string(i));

    plant.SetFreeBodyPose(
        &plant_context, box,
        drake::math::RigidTransformd{
            Eigen::Quaterniond(box_rotation[0], box_rotation[1],
                               box_rotation[2], box_rotation[3]),
            Vector3d(box_translation[0], box_translation[1],
                     box_translation[2])});
  }

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
