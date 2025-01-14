# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            trans,
            wait,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx(347.96, 94.11, 100.0, 10.62, -179.61, 11.19)
    pos2 = posx(400.54, 93.725, 100.0, 10.94, -179.45, 11.605)
    pos3 = posx(447.38, 93.34, 100.0, 11.26, -179.29, 12.02)
    pos4 = posx(346.92, 42.39, 100.0, 10.385, -179.35, 11.15)
    pos5 = posx(399.775, 42.46, 100.0, 12.105, -179.305, 12.8575)
    pos6 = posx(450.38, 42.53, 100.0, 13.825, -179.265, 14.565)
    pos7 = posx(345.88, -9.33, 100.0, 10.15, -179.08, 11.11)
    pos8 = posx(399.01, -8.805, 100.0, 13.27, -179.16, 14.11)
    pos9 = posx(447.38, -8.28, 100.0, 16.39, -179.24, 17.11)
    positions = [pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9]
    posmid = posx(719.607, -30.984, 281.545, 171.693, -156.091, 164.401)
    posmidj = posj(-1.463, 35.083, 34.039, -2.371, 87.111, -9.498)
    possleep1 = posj(-28.462, 82.843, 3.638, 88.788, 110.89, 2.829)
    possleep2 = posj(22.702, 64.999, 24.945, -71.176, 103.418, 1.291)
    
    zdown = [0, 0, -65, 0, 0, 0]
    xup = [150, 0, 0, 0, 0, 0]
    xdown = [-150, 0, 0, 0, 0, 0]
    xupzdown = [150, 0, -65, 0, 0, 0]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    while rclpy.ok():

        print("movej")
        movej(JReady, vel=VELOCITY, acc=ACC)
        for i in range(9):
            release()
            print(f"movel_pos{i}")
            movel(positions[i], vel=VELOCITY, acc=ACC)
            x1 = trans(positions[i], zdown)
            movel(x1, vel=VELOCITY, acc=ACC)
            grip()
            movel(positions[i], vel=VELOCITY, acc=ACC)
            movel(posmid, vel=VELOCITY, acc=10)
            wait(0.5)
            movej(possleep2, vel=30, acc=10)
            release()
            wait(0.5)
            movej(posmidj, vel = 20, acc=ACC)
            wait(0.5)
            movej(possleep1, vel=30, acc=ACC)
            grip()
            wait(0.5)
            movel(posmid, vel=VELOCITY, acc=ACC)
            x3 = trans(positions[i], xup)
            movel(x3, vel=VELOCITY, acc=ACC)
            x4 = trans(positions[i], xupzdown)
            movel(x4, VELOCITY,acc=ACC)
            release()
            x5 = trans(positions[i], xup)
            movel(x5, vel=VELOCITY, acc=ACC)

        rclpy.shutdown()


    rclpy.shutdown()
if __name__ == "__main__":
    main()
