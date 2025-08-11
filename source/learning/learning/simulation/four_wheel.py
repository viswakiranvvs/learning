
import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


BIPAD_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/DATA1/ai24mtech11006/ai24mtech11006/isaac_sim/learning/models/usd/four_wheel.usd",
        # rigid_props=sim_utils.RigidBodyPropertiesCfg(
        #     rigid_body_enabled=True,
        #     enable_gyroscopic_forces=True,
        #     disable_gravity=False,
        #     max_depenetration_velocity=5.0,            
        # ),
        # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        #     enabled_self_collisions=True,
        #     solver_position_iteration_count=4,
        #     solver_velocity_iteration_count=0,
        #     sleep_threshold=0.005,
        #     stabilization_threshold=0.001,
        # ),
    ),
    # actuators={}
    # joint_names=[".*", "^((?!torso_joint|hipjoint_r3|hipjoint_l3).)*$"],  # exclude these joints
    actuators={"actuators": ImplicitActuatorCfg(joint_names_expr=["LF", "LB","RF","RB"], stiffness=None, damping=None)},
)

# DOFBOT_CONFIG = ArticulationCfg(
#     spawn=sim_utils.UsdFileCfg(
#         usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Dofbot/dofbot.usd",
#         rigid_props=sim_utils.RigidBodyPropertiesCfg(
#             disable_gravity=False,
#             max_depenetration_velocity=5.0,
#         ),
#         articulation_props=sim_utils.ArticulationRootPropertiesCfg(
#             enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
#         ),
#     ),
#     init_state=ArticulationCfg.InitialStateCfg(
#         joint_pos={
#             "joint1": 0.0,
#             "joint2": 0.0,
#             "joint3": 0.0,
#             "joint4": 0.0,
#         },
#         pos=(0.25, -0.25, 0.0),
#     ),
#     actuators={
#         "front_joints": ImplicitActuatorCfg(
#             joint_names_expr=["joint[1-2]"],
#             effort_limit_sim=100.0,
#             velocity_limit_sim=100.0,
#             stiffness=10000.0,
#             damping=100.0,
#         ),
#         "joint3_act": ImplicitActuatorCfg(
#             joint_names_expr=["joint3"],
#             effort_limit_sim=100.0,
#             velocity_limit_sim=100.0,
#             stiffness=10000.0,
#             damping=100.0,
#         ),
#         "joint4_act": ImplicitActuatorCfg(
#             joint_names_expr=["joint4"],
#             effort_limit_sim=100.0,
#             velocity_limit_sim=100.0,
#             stiffness=10000.0,
#             damping=100.0,
#         ),
#     },
# )


class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot
    bipad = BIPAD_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Four_wheel_bot")
    # Dofbot = DOFBOT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Dofbot")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():
        # reset
        if count>=100 and count<=120:
        #     # reset counters
            # count = 0
            joint_indices = {name: i for i, name in enumerate(scene["bipad"].data.joint_names)}

            wave_action = scene["bipad"].data.default_joint_pos
            # print(wave_action)
            temp = 20
            wave_action[0, joint_indices["LF"]] = temp
            wave_action[0, joint_indices["LB"]] = temp
            wave_action[0, joint_indices["RF"]] = -temp
            wave_action[0, joint_indices["RB"]] = -temp
            # wave_action[0,2] = temp
            # wave_action[0,1] = -1*temp
            # wave_action[0,3:] = -1*temp
            print(wave_action)
            # scene["bipad"].set_joint_position_target(wave_action)
            scene["bipad"].set_joint_velocity_target(wave_action)
            scene.write_data_to_sim()
            sim.step()
            sim_time += sim_dt
            count += 1
            scene.update(sim_dt)
        # reset the scene entities to their initial positions offset by the environment origins
        #     root_jetbot_state = scene["bipad"].data.default_root_state.clone()
        #     root_jetbot_state[:, :3] += scene.env_origins
        #     print(root_jetbot_state)
        # #     root_dofbot_state = scene["Dofbot"].data.default_root_state.clone()
        # #     root_dofbot_state[:, :3] += scene.env_origins

        # #     # copy the default root state to the sim for the jetbot's orientation and velocity
        #     scene["bipad"].write_root_pose_to_sim(root_jetbot_state[:, :7])
        #     scene["bipad"].write_root_velocity_to_sim(root_jetbot_state[:, 7:])

            


            # scene.reset()
        #     scene["Dofbot"].write_root_pose_to_sim(root_dofbot_state[:, :7])
        #     scene["Dofbot"].write_root_velocity_to_sim(root_dofbot_state[:, 7:])

        #     # copy the default joint states to the sim
        #     joint_pos, joint_vel = (
        #         scene["Jetbot"].data.default_joint_pos.clone(),
        #         scene["Jetbot"].data.default_joint_vel.clone(),
        #     )
        #     scene["Jetbot"].write_joint_state_to_sim(joint_pos, joint_vel)
        #     joint_pos, joint_vel = (
        #         scene["Dofbot"].data.default_joint_pos.clone(),
        #         scene["Dofbot"].data.default_joint_vel.clone(),
        #     )
        #     scene["Dofbot"].write_joint_state_to_sim(joint_pos, joint_vel)
        #     # clear internal buffers
        #     scene.reset()
            print("[INFO]: Resetting state...")

        # # drive around
        # if count % 100 < 75:
        #     # Drive straight by setting equal wheel velocities
        #     action = torch.Tensor([[12.0, 12.0]])
        # else:
        #     # Turn by applying different velocities
        #     action = torch.Tensor([[3.0, -3.0]])

        # scene["Jetbot"].set_joint_velocity_target(action)

        # # wave
        else:
            joint_indices = {name: i for i, name in enumerate(scene["bipad"].data.joint_names)}

            wave_action = scene["bipad"].data.default_joint_pos
            # print(wave_action)
            # temp = 2 * np.sin(2 * np.pi * 0.5 * sim_time)
            temp = 10
            wave_action[0, joint_indices["LF"]] = temp
            wave_action[0, joint_indices["LB"]] = temp
            wave_action[0, joint_indices["RF"]] = temp
            wave_action[0, joint_indices["RB"]] = temp
            # wave_action[0,2] = temp
            # wave_action[0,1] = -1*temp
            # wave_action[0,3:] = -1*temp
            print(wave_action)
            # scene["bipad"].set_joint_position_target(wave_action)
            scene["bipad"].set_joint_velocity_target(wave_action)
            scene.write_data_to_sim()
            
            sim.step()
            sim_time += sim_dt
            count += 1
            scene.update(sim_dt)
        
        if count==120:
            count=0


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    # sim._world.physics_scene.set_gravity([0.0, 0.0, -9.81])

    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_cfg = NewRobotsSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # scene._scene._physics_scene.set_gravity([0.0, 0.0, -9.81])
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()