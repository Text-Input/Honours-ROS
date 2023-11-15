import random

bridge_template = "bridge.yaml.template"
world_template = "worlds/multi_agents6.world.template"

bridge_str = '''
- topic_name: "/model/{target}/pose"
  ros_type_name: "geometry_msgs/msg/Pose"
  gz_type_name: "gz.msgs.Pose"
  direction: GZ_TO_ROS
'''

world_str = '''
  <include>
    <uri>target</uri>
    <name>{target}</name>
    <pose>{pose} 0 -0 0</pose>
  </include>
'''

num_targets = 50
targets_box_x = (-50, 50)
targets_box_y = (-50, 50)
targets_box_z = (0.5, 0.5)

combined_bridges = ""
combined_worlds = ""
for i in range(num_targets):
    target_name = f"target{i}"

    x = random.uniform(targets_box_x[0], targets_box_x[1])
    y = random.uniform(targets_box_y[0], targets_box_y[1])
    z = random.uniform(targets_box_z[0], targets_box_z[1])
    pos = f"{x} {y} {z}"

    combined_bridges += bridge_str.replace("{target}", target_name)
    combined_worlds += world_str.replace("{target}", target_name).replace("{pose}", pos)


with open(bridge_template, 'r') as template, open(bridge_template.replace(".template", ""), "w") as output:
    text = template.read()
    text = text.replace("{}", combined_bridges)
    output.write(text)

with open(world_template, 'r') as template, open(world_template.replace(".template", ""), "w") as output:
    text = template.read()
    text = text.replace("{}", combined_worlds)
    output.write(text)
