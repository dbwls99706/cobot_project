# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

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
            # Robot motion APIs
            movej, movel, movesx, trans, amove_periodic,  # stop,
            # IO and force/torque APIs
            set_digital_output, get_digital_input,
            set_ref_coord, set_tool, set_tcp,
            task_compliance_ctrl, set_stiffnessx,
            set_desired_force, release_force,
            release_compliance_ctrl,
            check_force_condition,
            set_velj, set_accj, set_velx, set_accx,
            set_singular_handling,
            wait,
            get_current_posx,
            # wait_digital_input,
            DR_AXIS_Z, DR_BASE, DR_TOOL,
            DR_FC_MOD_REL, DR_MV_RA_DUPLICATE, DR_MV_APP_NONE, DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_AVOID, DR_QSTOP,
            DR_MVS_VEL_NONE,
            DR_MVS_VEL_CONST,
            amovesx
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
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    JReady = [0, 0, 90, 0, 90, 0]
    grapos = [329.476, 166.934, 322.79, 102.228, -177.288, 104.871]
    moving = [-1,-300, 0, 0, 0, 0]
    turn1 = [299.245, -69.872, 45.0, 88.462, 120.0, 91.837]
    zdown = [0, 0, -60, 0, 0, 0]
    zup = [0, 0, 120, 0, 0, 0]
    zup2 = [0, 0, 30, 0, 0, 0]
    zup3 = [0, 0, 70, 0, 0, 0]
    yup = [0, -100, 0, 0, 0, 0]
    pos_grap = [290.145, -31.508, 29.37, 158.888, 178.24, 157.822] #반대쪽 잡기(90도로 )
    pos_set = [311.786, -69.325, 99.546, 93.231, 91.182, 88.995] #세워놓기
    pos_last_grip = [312.664, -70.469, 70.773, 5.386, -178.325, 1.379] #다시 잡기
    
    swing1 = posx(291.193, 19.999, 177.007, 173.597, 164.148, -161.381)

    swing_mid = posx(401.367, -28.416, 142.794, 103.116, -173.638, 127.718)

    swing2 = posx(498.097, -58.778, 164.321, 146.752, -158.843, 168.79)
    swing_final = posx(519.013, -71.132, 169.984, 147.008, -155.181, 170.125)
    swing_list = [swing1, swing_mid, swing2]
    swing_rev = [swing2, swing_mid, swing1]
    swing_fin = [swing1, swing_mid, swing_final]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        
        release()
        print("I'm Ready")
        movej(JReady, vel=VELOCITY, acc=ACC)
        movel(grapos, vel=VELOCITY, acc=ACC)
        grip()
        movel(zup2,vel=VELOCITY, acc=ACC, mod = DR_MV_MOD_REL)
        movel(yup,vel=VELOCITY, acc=ACC, mod = DR_MV_MOD_REL)
        movel(turn1, vel=VELOCITY, acc=ACC)     
        release()
        
        movel(pos_grap, vel=VELOCITY, acc=ACC)
        grip()
        movel(zup,vel=VELOCITY, acc=ACC, mod = DR_MV_MOD_REL)
        movel(pos_set, vel=VELOCITY, acc=ACC)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        release_compliance_ctrl()
        release()
        movel(zup3,vel=VELOCITY, acc=ACC, mod = DR_MV_MOD_REL)
        movel(pos_last_grip, vel=VELOCITY, acc=ACC)
        grip()
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        grip()
        movel(swing1, vel=VELOCITY, acc=ACC)
        for i in range(2):
            movesx(swing_list, vel=[200, 60], acc=[100, 60], time = 1.5, vel_opt=DR_MVS_VEL_NONE)
            movesx(swing_rev, vel=[200, 60], acc=[100, 60], time = 1.5, vel_opt=DR_MVS_VEL_NONE)
        amovesx(swing_fin, vel=[250, 150], acc=[100, 60], time = 0.8, vel_opt=DR_MVS_VEL_NONE)
        wait(0.7)
        release()
        rclpy.shutdown()
if __name__ == "__main__":
    main()