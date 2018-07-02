#!/usr/bin/python

import sys

filename = "generated.launch";
el = '\n'  # endline
tab = '\t'

amiroids = []
imageselectornr = 1
camid = 3


def writeLaunchBegin(file):
  file.write("""<?xml version="1.0"?>""" + el + '<launch>' + el)


def writeLaunchEnd(file):
  file.write(el + el + '</launch>' + el)


def writeRviz(file):
  file.write(el + tab + '<node name="rviz" pkg="rviz" type="rviz" args="-d $(find potential_field_navigation)/config/config.rviz" output="screen" respawn="true"/>' + el)


def writeImageViewerNode(file, topic1, topic2):
  file.write(el + tab + '<node name="image_viewer_node" pkg="potential_field_navigation" type="image_viewer_node" output="screen" cwd="ROS_HOME">' + el
             + tab + tab + '<param name="subcriber_topic_1" value="' + topic1 + '"/>' + el
             + tab + tab + '<param name="subcriber_topic_2" value="' + topic2 + '"/>' + el
             + tab + '</node>' + el)


def writeImageSelectorNode(file, id):
  file.write(el + tab + '<node name="image_selecter_node' + str(id) + '" pkg="potential_field_navigation" type="image_selecter_node" output="screen" cwd="ROS_HOME">' + el
             + tab + tab + '<param name="image_publisher_topic" value="/image' + str(id) + '"/>' + el
             + tab + '</node>' + el)


def writeImageToVectorfieldNode(file, id):
  file.write(el + tab + '<node name="image' + str(id) + '_to_vectorfield_node" pkg="potential_field_navigation" type="image_to_vectorfield_node" output="screen" cwd="ROS_HOME">' + el
             + tab + tab + '<param name="image_listener_topic" value="/image' + str(id) + '"/>' + el
             + tab + tab + '<param name="potentialfield_publisher_topic" value="/potentialfield/image' + str(id) + '"/>' + el
             + tab + tab + '<param name="vectorfield_publisher_topic" value="/vectorfield/image' + str(id) + '"/>' + el
             + tab + tab + '<param name="heuristic_factor" value="1.0"/> <?Maximum vector lengh?>' + el
             + tab + tab + '<param name="heuristic_abs_min" value="0.8"/> <?Undefined An arbitrary value which indecates the start of value decay?>' + el
             + tab + tab + '<param name="heuristic_apply" value="1"/> <?Use heuristic instead of native vectorfield?>' + el
             + tab + tab + '<param name="desired_vectorfield_width" value="$(arg image_width)"/>' + el
             + tab + tab + '<param name="desired_vectorfield_height" value="$(arg image_height)"/>' + el
             + tab + '</node>' + el)


def writeVectorfieldToGridmapNode(file, id, topic1, topic2 = 0):
  if (topic2 == 0):
    topic2 = topic1
  file.write(el + tab + '<node name="vectorfield_to_gridmap_node' + str(id) + '" pkg="potential_field_navigation" type="vectorfield_to_gridmap_node" output="screen" cwd="ROS_HOME">' + el
             + tab + tab + '<param name="meter_per_pixel" value="$(arg meter_per_pixel)"/>' + el
             + tab + tab + '<param name="frame_id" value="$(arg cam)_image"/>' + el
             + tab + tab + '<param name="vectorfield_listener_topic" value="/vectorfield/' + topic1 + '"/>' + el
             + tab + tab + '<param name="gridmap_publisher_topic" value="/vectorfield/' + topic2 + '/gridmap"/>' + el
             + tab + '</node>' + el)


def writeTransforms(file):
  file.write(el + tab + '<node name="tf_world_cam" pkg="tf" type="static_transform_publisher" args="0 0 3.66265 0 0 3.14159 world $(arg cam) 100"/>' + el
             + tab + '<node name="tf_world_cam_image" pkg="tf" type="static_transform_publisher" args="0 0 0 1.5708 0 0 world $(arg cam)_image 100"/>' + el)


def writeAmiroRosToRsbBridges(file, id):
  file.write(el + tab + '<node name="AMiRo' + str(id) + '" pkg="ros_to_rsb_bridge" type="ros_geometry_msgs_twist_to_rst_value_array" output="screen" cwd="ROS_HOME">' + el
             + tab + tab + '<param name="ros_listener_topic" value="/amiro' + str(id) + '/cmd_vel"/>' + el
             + tab + tab + '<param name="rsb_publish_scope" value="/amiro' + str(id) + '/motor/5"/>' + el
             + tab + tab + '<param name="duration" value="1000000"/>' + el
             + tab + '</node>' + el)


def writeOdomToVectorfieldNode(file, id):
  file.write(el + tab + '<node name="odom' + str(id) + '_to_vectorfield_node" pkg="potential_field_navigation" type="odom_to_vectorfield_node" output="screen" cwd="ROS_HOME">' + el
             + tab + tab + '<param name="meter_per_pixel" value="$(arg meter_per_pixel)"/>' + el
             + tab + tab + '<param name="frame_id" value="world"/>' + el
             + tab + tab + '<!--<param name="amiro_odom_listener_topic" value="$(arg tracker_prefix)/odom/' + str(id) + '"/>-->' + el
             + tab + tab + '<param name="amiro_odom_listener_topic" value="$(arg tracker_prefix)/pixel/' + str(id) + '" if="$(arg pixel_mode)"/>' + el
             + tab + tab + '<param name="potentialfield_publisher_topic" value="/potentialfield/amiro' + str(id) + '"/>' + el
             + tab + tab + '<param name="vectorfield_publisher_topic" value="/vectorfield/amiro' + str(id) + '"/>' + el
             + tab + tab + '<param name="pixel_mode" value="$(arg pixel_mode)"/>' + el
             + tab + tab + '<param name="pixel_scale" value="$(arg pixel_scale)"/>' + el
             + tab + tab + '<param name="image_width" value="$(arg image_width)"/>' + el
             + tab + tab + '<param name="image_height" value="$(arg image_height)"/>' + el
             + tab + tab + '<param name="heuristic_apply" value="1"/>' + el
             + tab + tab + '<param name="heuristic_abs_min" value="$(arg amiro_heuristic_abs_min)"/> <?Undefined An arbitrary value which indecates the start of value decay?>' + el
             + tab + tab + '<param name="heuristic_factor" value="$(arg amiro_heuristic_factor)"/> <?Maximum vector lengh?>' + el
             + tab + tab + '<param name="minimum_pose_difference_pixel" value="$(arg amiro_minimum_pose_difference_pixel)"/> <?Only recalculate the map if the robot has moved that much?>' + el
             + tab + '</node>' + el)


def writeVectorfieldMerger(file, id, topic1, topic2, topicResult, normalize = 0):
  file.write(el + tab + '<node name="vectorfield_merger_' + str(id) + '" pkg="potential_field_navigation" type="vectorfield_merger_node" output="screen" cwd="ROS_HOME">' + el
             + tab + tab + '<param name="field1_listener_topic" value="/vectorfield/' + topic1 + '"/>' + el
             + tab + tab + '<param name="field2_listener_topic" value="/vectorfield/' + topic2 + '"/>' + el
             + tab + tab + '<param name="field_width" value="$(arg image_width)"/>' + el
             + tab + tab + '<param name="field_height" value="$(arg image_height)"/>' + el
             + tab + tab + '<param name="vectorfield_publisher_topic" value="/vectorfield/' + topicResult + '"/>' + el
             + tab + tab + '<param name="normalize" value="' + str(normalize) + '"/>' + el
             + tab + '</node>' + el)


def writeVectorfieldToKinematicNode(file, id, vectorfieldTopic):
  file.write(el + tab + '<node name="amiro' + str(id) + '_vectorfield_to_kinematic_node" pkg="potential_field_navigation" type="vectorfield_to_kinematic_node" output="screen" cwd="ROS_HOME">' + el
             + tab + tab + '<param name="meter_per_pixel" value="$(arg meter_per_pixel)"/>' + el
             + tab + tab + '<param name="vectorfield_listener_topic" value="/vectorfield/' + vectorfieldTopic + '"/>' + el
             + tab + tab + '<!--<param name="amiro_odom_listener_topic" value="/artoolkit5_$(arg cam)/cam/odom/' + str(id) + '" unless="$(arg pixel_mode)"/>-->' + el
             + tab + tab + '<param name="amiro_odom_listener_topic" value="$(arg tracker_prefix)/pixel/' + str(id) + '" if="$(arg pixel_mode)" />' + el
             + tab + tab + '<param name="twist_publisher_topic" value="/amiro' + str(id) + '/cmd_vel"/>' + el
             + tab + tab + '<param name="velocityScale_meterPerSecond" value="0.1"/>' + el
             + tab + tab + '<param name="angularScale_radPerSecond" value="0.5"/>' + el
             + tab + tab + '<param name="pixel_mode" value="$(arg pixel_mode)"/>' + el
             + tab + tab + '<param name="pixel_scale" value="$(arg pixel_scale)"/>' + el
             + tab + tab + '<param name="twist_mode" value="True"/>' + el
             + tab + tab + '<param name="frequency" value="$(arg amiro_cmd_vel_frequency)"/>' + el
             + tab + '</node>' + el)


def writeHeader(file, name):
  file.write(el + el + el + tab + '<!-- ' + name + ' -->')


if __name__ == "__main__":

  if (len(sys.argv) < 4 or sys.argv[1] == "help" or sys.argv[1] == "-h" or sys.argv[1] == "--help"):
    print("Following usage of this programm: ./launchfile_generator.py <camid> <number of image_selecter_nodes> <amiroid1> <amiroid2> <amiroid_X>")
    exit(0)

  camid = int(sys.argv[1])
  imageselectornr = int(sys.argv[2])
  if (imageselectornr > 2):
    print("imageselectornr has to be lesser than 3.")
    exit(0)
  if (imageselectornr < 1):
    print("imageselectornr has to be greater than 0.")
    exit(0)

  for x in range(3, len(sys.argv)):
    amiroids.append(int(sys.argv[x]))
  amiroids.sort()
  filename = "camid_" + str(camid) + "_imageselectornodes_" + str(imageselectornr) + "_amiroids_" + str(''.join(str(e) for e in amiroids)) + "_generated.launch"
  print("filename: " + str(filename))
  print("camid: " + str(camid))
  print("number of image_selector_nodes: " + str(imageselectornr))
  print("amiroids: " + str(amiroids))
  if (any(amiroids.count(x) > 1 for x in amiroids)):
    print "There are duplicate amiroids."
    exit(0)
  if (any(x < 0 for x in amiroids)):
    print "There are negative amiroids."
    exit(0)

  argslist = [
    """<param name="use_sim_time" value="false"/>""",
    """<arg name="rviz" default="true" />""",
    """<arg name="cam" default="cam""" + str(camid) + """_2"/>""",
    """<arg name="meter_per_pixel_original" value="0.0023"/>""",
    """<arg name="image_width_original" value="2560"/>""",
    """<arg name="image_height_original" value="2048"/>""",
    """<arg name="pixel_scale" value="0.1"/>""",
    """<arg name="pixel_mode" value="true"/>""",
    """<arg name="meter_per_pixel" value="$(eval arg('meter_per_pixel_original') / arg('pixel_scale'))"/>""",
    """<arg name="image_width" value="$(eval arg('image_width_original') * arg('pixel_scale'))"/>""",
    """<arg name="image_height" value="$(eval arg('image_height_original') * arg('pixel_scale'))"/>""",
    """<arg name="tracker_prefix" value="/artoolkit5_cam""" + str(camid) + """_2/cam"/>""",
    """<arg name="amiro_heuristic_abs_min" value="0.4"/>""",
    """<arg name="amiro_heuristic_factor" value="0.5"/>""",
    """<arg name="amiro_minimum_pose_difference_pixel" value="2"/>""",
    """<arg name="amiro_cmd_vel_frequency" value="10"/>"""
  ]

  with open(filename, 'w+') as file:
    writeLaunchBegin(file)
    for x in argslist:
      file.write(tab + x + el)

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Rviz")
    writeRviz(file)

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Image Viewer Node")
    writeImageViewerNode(file, "/genicam_cam" + str(camid) + "/image_raw", "/vectorfield/fused/gridmap/rgb_image")

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Image Selector Nodes")
    for x in range(1, imageselectornr + 1):
      writeImageSelectorNode(file, x)

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Image to Vectorfield Nodes")
    for x in range(1, imageselectornr + 1):
      writeImageToVectorfieldNode(file, x)

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Vectorfield to GridMap")
    if (imageselectornr == 1):
      writeVectorfieldToGridmapNode(file, "_image1", "image1")
    else:
      writeVectorfieldToGridmapNode(file, "_image", "image")

    for id in amiroids:
      writeVectorfieldToGridmapNode(file, "_amiro" + str(id), "amiro" + str(id))

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Transform")
    writeTransforms(file)

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Bridges")
    for id in amiroids:
      writeAmiroRosToRsbBridges(file, id)

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Amiro Vectorfield Creator")
    for id in amiroids:
      writeOdomToVectorfieldNode(file, id)

    ###################################################################################
    ###################################################################################

    if (imageselectornr == 2):
      writeHeader(file, "Vectorfield Merger Images")
      writeVectorfieldMerger(file, "_image1_image2", "image1", "image2", "image")

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Vectorfield Merger Amiros")
    mergerTopics = []
    odd = True
    if (len(amiroids) > 1):
      for i in range(0, len(amiroids), 2):
        if (i + 1 == len(amiroids)):
          # print("amiroids not assigned: " + str(amiroids[i]))
          odd = False
          break
        resultTopic = str(amiroids[i]) + str(amiroids[i + 1])
        mergerTopics.append(resultTopic)
        writeVectorfieldMerger(file, str(amiroids[i]) + "_" + str(amiroids[i + 1]), "amiro" + str(amiroids[i]), "amiro" + str(amiroids[i + 1]), resultTopic + "/fused")

      if (odd == False):
        resultTopic = str(amiroids[-1]) + mergerTopics[0]
        writeVectorfieldMerger(file, str(amiroids[-1]) + "_" + mergerTopics[0], "amiro" + str(amiroids[-1]), mergerTopics[0] + "/fused", resultTopic + "/fused")
        mergerTopics.remove(mergerTopics[0])
        mergerTopics.append(resultTopic)

      # print(mergerTopics)
      merging = True
      mergerTopicsTemp = []
      while (merging):
        if (len(mergerTopics) == 1):
          merging = False
          break
        for i in range(0, len(mergerTopics), 2):
          if (i + 1 == len(mergerTopics)):
            # print(mergerTopics[i])
            break
          # print(mergerTopics[i] + " " + mergerTopics[i+1])
          resultTopic = mergerTopics[i] + mergerTopics[i + 1]
          writeVectorfieldMerger(file, mergerTopics[i] + "_" + mergerTopics[i + 1], mergerTopics[i] + "/fused", mergerTopics[i + 1] + "/fused", resultTopic + "/fused")
          mergerTopicsTemp.append(resultTopic)
        mergerTopics = mergerTopicsTemp

    finalVectorfieldScope = ""
    imageScope = "image"

    if (imageselectornr == 1):
      imageScope = "image1"

    if (len(amiroids) == 1):
      mergedVectorfieldScope = "amiro" + str(amiroids[0])
      finalVectorfieldScope = mergedVectorfieldScope + "_image/fused"
      writeVectorfieldMerger(file, mergedVectorfieldScope, mergedVectorfieldScope, imageScope, finalVectorfieldScope, 1)
    else:
      finalVectorfieldScope = mergerTopics[0] + "image" + "/fused"
      writeVectorfieldMerger(file, mergerTopics[0] + "_image", mergerTopics[0] + "/fused", imageScope, finalVectorfieldScope, 1)

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Vectorfield to GridMap Final")
    writeVectorfieldToGridmapNode(file, 0, finalVectorfieldScope, "fused")

    ###################################################################################
    ###################################################################################

    writeHeader(file, "Steering")
    for id in amiroids:
      writeVectorfieldToKinematicNode(file, id, finalVectorfieldScope)

    ###################################################################################
    ###################################################################################

    # writeImageToVectorfieldNode(file, 1)
    # writeImageToVectorfieldNode(file, 2)
    # writeVectorfieldToGridmapNode(file, 1, "image")
    # writeTransforms(file)
    # writeAmiroRosToRsbBridges(file, 1)
    # writeAmiroRosToRsbBridges(file, 2)
    # writeOdomToVectorfieldNode(file, 1)
    # writeOdomToVectorfieldNode(file, 2)
    # writeVectorfieldMerger(file, "awdawd", "topic1", "topic2", "topic1"+"topic2")
    # writeVectorfieldToKinematicNode(file, 1, "12image/fused")
    # writeVectorfieldToKinematicNode(file, 2, "12image/fused")

    writeLaunchEnd(file)
