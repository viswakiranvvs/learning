# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# from isaacsim import SimulationApp

# # This sample enables a livestream server to connect to when running headless
# CONFIG = {
#     "width": 1280,
#     "height": 720,
#     "window_width": 1920,
#     "window_height": 1080,
#     "headless": True,
#     "hide_ui": False,  # Show the GUI
#     "renderer": "RaytracedLighting",
#     "display_options": 3286,  # Set display options to show default grid
# }


# # Start the omniverse application
# kit = SimulationApp(launch_config=CONFIG)

# from isaacsim.core.utils.extensions import enable_extension

# # Default Livestream settings
# kit.set_setting("/app/window/drawMouse", True)

# # Enable Livestream extension
# enable_extension("omni.kit.livestream.webrtc")

# # Run until closed
# while kit._app.is_running() and not kit.is_exiting():
#     # Run in realtime mode, we don't specify the step size
#     kit.update()

# kit.close()

import argparse

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

from isaaclab.sim import SimulationCfg, SimulationContext


def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = SimulationCfg(dt=0.01)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
