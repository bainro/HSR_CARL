import hsrb_interface

robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')

# raise hand
head_pan_joint_val = whole_body.joint_positions['head_pan_joint']
omni_base.go_rel(0, 0, .5, 100)
whole_body.move_to_joint_positions({'head_tilt_joint': .1})
head_tilt_joint_val = whole_body.joint_positions['head_tilt_joint']
arm_lift_joint_val = whole_body.joint_positions['arm_lift_joint']
wrist_flex_joint_val = whole_body.joint_positions['wrist_flex_joint']
whole_body.move_to_joint_positions({'arm_lift_joint': 0.67, 'wrist_flex_joint': 0, 'head_pan_joint': -.59})


# # lower hand
# whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_joint_val,'wrist_flex_joint':wrist_flex_joint_val,'head_pan_joint':head_pan_joint_val})
# whole_body.move_to_joint_positions({'head_tilt_joint':head_tilt_joint_val})
# omni_base.go_rel(0,0,-.5,100)
# whole_body.move_to_joint_positions({'head_tilt_joint':head_tilt_joint_val})
