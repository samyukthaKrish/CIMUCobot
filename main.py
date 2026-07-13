from draw_circle import CircleDrawer
from draw_box import BoxDrawer
from draw_lineInBox import LineInBoxDrawer

def main():
    robot_ip = '192.168.1.205'

    # 1. Initialize the tools you need
    circle_tool = CircleDrawer(robot_ip)
    box_tool = BoxDrawer(robot_ip)
    line_tool = LineInBoxDrawer(robot_ip)

    # 2. Prepare the robot using any of the tools 
    circle_tool.go_to_safe_home()

    # 3. Draw a Circle
    circle_tool.draw_circle(center_x=300.0, center_y=0.0, center_z=30, radius=40.0)

    # 4. Draw a Box
    box_tool.draw_box(start_x=250.0, start_y=-100.0, start_z=180.0, length=100.0, width=150.0)

    # 5. Draw a Centered Line in that Box boundary space
    line_tool.draw_lineInBox(
        start_x=250.0, start_y=-100.0, start_z=180.0,
        length=100.0, width=150.0, line_length=50.0, line_z=180.0
    )

    # Clean up
    circle_tool.go_to_safe_home()

if __name__ == "__main__":
    main()
