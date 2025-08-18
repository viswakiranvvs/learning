# import carb
from isaacsim import SimulationApp
import sys

BACKGROUND_STAGE_PATH = "/background"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of stages and manual publishing of images
simulation_app = SimulationApp(CONFIG)
import omni
import numpy as np
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import stage, extensions, nucleus
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import isaacsim.core.utils.prims as prims_utils
from isaacsim.core.utils.prims import set_targets,get_prim_at_path
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.nodes.scripts.utils import set_target_prims
# from isaacsim.asset.importer.urdf import URDFImporter
# # from isaaclab.assets.articulation import ArticulationCfg
# # from omni.isaac.urdf import _urdf
# extensions.enable_extension("omni.importer.urdf")
import omni.usd
# from omni.isaac.urdf.importer import URDFImporter
# import omni.isaac.core.utils.prims as prim_utils
# Enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")
from isaacsim.ros2.bridge import read_camera_info

simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    # carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)
# urdf_path = "/home/robot2/Documents/isaac_sim/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf"
# get_prim_at_path("/home/robot2/Documents/isaac_sim/mounted_turtlebot.usd","turtlebot3_burger")
# # Where to put the robot in the scene (USD prim path)
# robot_prim_path = "/World/Turtlebot3"

# importer = URDFImporter()
# Define a container prim for the asset
# prims_utils.define_prim("/World/turtlebot3_burger", prim_type="Xform")

# Reference in an external USD
omni.kit.commands.execute(
    "IsaacSimSpawnPrim",
    usd_path="/home/robot2/Documents/isaac_sim/mounted_turtlebot.usd",
    prim_path="/World/turtlebot3_burger",
    translation=(0, 0, 0),
    rotation=(0, 0, 0, 0),
)

# turtlebot = prims_utils.get_prim_at_path("/World/turtlebot3_burger")

# # Import robot
# prim = importer.import_robot(
#     file_path=urdf_path,
#     import_inertia_tensor=True,
#     fix_base=False,                     # mobile robot
#     make_instanceable=False,
#     default_drive_type="velocity",
#     prim_path=robot_prim_path
# )
###### Camera helper functions for setting up publishers. ########

# Paste functions from the tutorial here
# def publish_camera_tf(camera: Camera): ...
# def publish_camera_info(camera: Camera, freq): ...
# def publish_pointcloud_from_depth(camera: Camera, freq): ...
# def publish_depth(camera: Camera, freq): ...
# def publish_rgb(camera: Camera, freq): ...
def publish_camera_tf(camera: Camera):
    camera_prim = camera.prim_path

    if not is_prim_path_valid(camera_prim):
        raise ValueError(f"Camera path '{camera_prim}' is invalid.")

    try:
        # Generate the camera_frame_id. OmniActionGraph will use the last part of
        # the full camera prim path as the frame name, so we will extract it here
        # and use it for the pointcloud frame_id.
        camera_frame_id=camera_prim.split("/")[-1]

        # Generate an action graph associated with camera TF publishing.
        ros_camera_graph_path = "/CameraTFActionGraph"

        # If a camera graph is not found, create a new one.
        if not is_prim_path_valid(ros_camera_graph_path):
            (ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": ros_camera_graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                    ]
                }
            )

        # Generate 2 nodes associated with each camera: TF from world to ROS camera convention, and world frame.
        og.Controller.edit(
            ros_camera_graph_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("PublishTF_"+camera_frame_id, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("PublishRawTF_"+camera_frame_id+"_world", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTF_"+camera_frame_id+".inputs:topicName", "/tf"),
                    # Note if topic_name is changed to something else besides "/tf",
                    # it will not be captured by the ROS tf broadcaster.
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:topicName", "/tf"),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:parentFrameId", camera_frame_id),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:childFrameId", camera_frame_id+"_world"),
                    # Static transform from ROS camera convention to world (+Z up, +X forward) convention:
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
                ],
                og.Controller.Keys.CONNECT: [
                    (ros_camera_graph_path+"/OnTick.outputs:tick",
                        "PublishTF_"+camera_frame_id+".inputs:execIn"),
                    (ros_camera_graph_path+"/OnTick.outputs:tick",
                        "PublishRawTF_"+camera_frame_id+"_world.inputs:execIn"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime",
                        "PublishTF_"+camera_frame_id+".inputs:timeStamp"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime",
                        "PublishRawTF_"+camera_frame_id+"_world.inputs:timeStamp"),
                ],
            },
        )
    except Exception as e:
        print(e)

    # Add target prims for the USD pose. All other frames are static.
    set_target_prims(
        primPath=ros_camera_graph_path+"/PublishTF_"+camera_frame_id,
        inputName="inputs:targetPrims",
        targetPrimPaths=[camera_prim],
    )
    return

def publish_depth(camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = camera.name+"_depth"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                            sd.SensorType.DistanceToImagePlane.name
                        )
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return

def publish_rgb(camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = camera.name+"_rgb"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return

def publish_pointcloud_from_depth(camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = camera.name+"_pointcloud" # Set topic name to the camera's name
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

    # Note, this pointcloud publisher will convert the Depth image to a pointcloud using the Camera intrinsics.
    # This pointcloud generation method does not support semantic labeled objects.
    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )

    writer = rep.writers.get(rv + "ROS2PublishPointCloud")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return

def publish_camera_info(camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = camera.name+"_camera_info"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

    writer = rep.writers.get("ROS2PublishCameraInfo")
    camera_info = read_camera_info(render_product_path=render_product)
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name,
        width=camera_info["width"],
        height=camera_info["height"],
        projectionType=camera_info["projectionType"],
        k=camera_info["k"].reshape([1, 9]),
        r=camera_info["r"].reshape([1, 9]),
        p=camera_info["p"].reshape([1, 12]),
        physicalDistortionModel=camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product
    )

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return
###################################################################
def turtlebot_movement():
    graph_path = "/World/turtlebot_drive"
    if not is_prim_path_valid(graph_path):
        (graph, _, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution","pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("on_tick", "omni.graph.action.OnPlaybackTick"),
                        ("ros_context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("sub_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                        ("break_lin", "omni.graph.nodes.BreakVector3"),
                        ("break_ang", "omni.graph.nodes.BreakVector3"),
                        ("scale_units", "isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit"),
                        ("diff_ctrl", "isaacsim.robot.wheeled_robots.DifferentialController"),
                        ("artic_ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
                        ("make_array", "omni.graph.nodes.ConstructArray"),
                        ("append_array", "omni.graph.nodes.ArrayAppendValue"),
                        ("token0", "omni.graph.nodes.ConstantToken"),
                        ("token1", "omni.graph.nodes.ConstantToken"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        # ("make_array.inputs:arraySize", 2),
                        ("ros_context.inputs:domain_id", 0),
                        ("ros_context.inputs:useDomainIDEnvVar", True),
                        ("sub_twist.inputs:topicName", "/cmd_vel"),
                        ("sub_twist.inputs:queueSize", 10),
                        ("diff_ctrl.inputs:maxAngularSpeed",0.8),
                        ("diff_ctrl.inputs:maxLinearSpeed",0.4),
                        ("diff_ctrl.inputs:wheelDistance",0.16),
                        ("diff_ctrl.inputs:wheelRadius",0.025),
                        # ("sub_twist.inputs:qosProfile", "sensor_data"),
                        ("token0.inputs:value", "a___namespace_wheel_left_joint"),
                        ("token1.inputs:value", "a___namespace_wheel_right_joint"),
                        ("artic_ctrl.inputs:targetPrim", "/World/turtlebot3_burger/turtlebot3_burger"),
                        # ("make_array.inputs:input0", "a___namespace_wheel_left_joint"),
                        # ("make_array.inputs:input1", "a___namespace_wheel_right_joint"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("on_tick.outputs:tick", "sub_twist.inputs:execIn"),
                        ("on_tick.outputs:tick", "diff_ctrl.inputs:execIn"),
                        ("on_tick.outputs:tick", "artic_ctrl.inputs:execIn"),
                        ("on_tick.outputs:deltaSeconds", "diff_ctrl.inputs:dt"),
                        ("ros_context.outputs:context", "sub_twist.inputs:context"),
                        ("sub_twist.outputs:linearVelocity", "scale_units.inputs:value"),
                        ("sub_twist.outputs:angularVelocity", "break_ang.inputs:tuple"),
                        ("break_lin.outputs:x", "diff_ctrl.inputs:linearVelocity"),
                        ("break_ang.outputs:z", "diff_ctrl.inputs:angularVelocity"),
                        ("diff_ctrl.outputs:velocityCommand", "artic_ctrl.inputs:velocityCommand"),
                        ("scale_units.outputs:result", "break_lin.inputs:tuple"),
                        ("token0.inputs:value", "make_array.inputs:input0"),
                        ("token1.inputs:value", "append_array.inputs:value"),
                        ("make_array.outputs:array", "append_array.inputs:array"),
                        ("append_array.outputs:array", "artic_ctrl.inputs:jointNames"),
                        # ("token0.inputs:value", "make_array.inputs:input0"),
                        
                        # ("token1.inputs:value", "make_array.inputs:input1"),
                    ],
                },
        )
        # og.Controller.evaluate()
    # make_array_node = og.Controller.node("/World/turtlebot_drive/make_array")
    # og.Controller.attribute("inputs:input0", make_array_node)
    # og.Controller.attribute("inputs:input1", make_array_node)
    # else:
    # og.Controller.evaluate(graph_path)
    # og.Controller.edit(graph_path,
    #                 {
    #                     og.Controller.Keys.CONNECT: [
    #                         (graph_path+"/token1.inputs:value", graph_path+"/make_array.inputs:input1"),
    #                     ]
    # })
    # og.Controller.connect("/World/turtlebot_drive/token1.inputs:value", "/World/turtlebot_drive/make_array.inputs:input1")
    # og.Controller.edit(
    #     graph_path,{
    #     og.Controller.Keys.CONNECT: [
    #                 ("token1.inputs:value", "make_array.inputs:input1"),
    #             ],
    #     }
    # )
# Create a Camera prim. Note that the Camera class takes the position and orientation in the world axes convention.
camera = Camera(
    prim_path="/World/floating_camera",
    position=np.array([-3.11, -1.87, 1.0]),
    frequency=20,
    resolution=(720, 720),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
)
camera.initialize()

simulation_app.update()
camera.initialize()

depthCameraPrim = prims_utils.get_prim_at_path("/World/turtlebot3_burger/turtlebot3_burger/rsd455/RSD455/Camera_Pseudo_Depth")
depthCamera = Camera(
    prim_path="/World/turtlebot3_burger/turtlebot3_burger/rsd455/RSD455/Camera_Pseudo_Depth",
    resolution=(640, 480),
)
# depthCamera.set_resolution((640, 480))
depthCamera.initialize()

rgbCamera = Camera(
    prim_path="/World/turtlebot3_burger/turtlebot3_burger/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
    resolution=(640, 480),
)
# rgbCamera.set_resolution((640, 480))
rgbCamera.initialize()
############### Calling Camera publishing functions ###############

# Call the publishers.
# Make sure you pasted in the helper functions above, and uncomment out the following lines before running.

approx_freq = 30
publish_camera_tf(rgbCamera)
publish_camera_info(rgbCamera, approx_freq)
publish_rgb(rgbCamera, approx_freq)
publish_depth(depthCamera, approx_freq)
publish_pointcloud_from_depth(depthCamera, approx_freq)
turtlebot_movement()
####################################################################

# Initialize physics
simulation_context.initialize_physics()
simulation_context.play()

while simulation_app.is_running():
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()