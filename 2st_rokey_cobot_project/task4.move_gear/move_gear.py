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
            DR_TOOL,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            DR_BASE,
            amove_periodic,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def get_current_pos():
        """현재 로봇의 위치를 반환하는 함수."""
        try:
            from DSR_ROBOT2 import get_current_posx
            pos = get_current_posx()
            return pos
        except ImportError as e:
            print(f"Error importing DSR_ROBOT2: {e}")
            return None
    
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    JReady = [0, 0, 90, 0, 90, 0]
    gear1 = posx(397.385, 91.27, 100, 57.989, -179.774, 58.811)
    gear2 = posx(486.44, 149.771, 100, 97.714, -179.749, 98.343)
    gear3 = posx(392.944, 196.498, 100, 147.592, -179.755, 147.833)
    gear_mid = posx(426.776, 144.465, 100, 119.403, -179.69, 119.645)
    gears = [gear1, gear2, gear3, gear_mid]
    moving = [-1,-300, 0, 0, 0, 0]
    
    zdown = [0, 0, -60, 0, 0, 0]
    zdown2 = [0, 0, -20, 0, 0, 0]
    zput = [0, 0, -50, 0, 0, 0] 
    zup = [0, 0, 60, 0, 0, 0]
    zup2 = [0, 0, 50, 0, 0, 0]
    xup = [150, 0, 0, 0, 0, 0]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        release()
        print("I'm Ready")
        movej(JReady, vel=VELOCITY, acc=ACC)

        for i in range (3):
            movel(gears[i], vel=VELOCITY, acc= ACC)
            movel(zdown,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
            grip()
            movel(zup,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
            movel(moving,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
            movel(zput,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
            release()
            movel(zup2,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
               
        movel(gear_mid, vel = VELOCITY, acc= ACC)
        movel(zdown,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
        grip()
        movel(zup,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
        movel(moving,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
        movel(zdown2,  vel=VELOCITY, acc= ACC, mod=DR_MV_MOD_REL)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -40, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
        amove_periodic(amp=[0, 0, 0, 0, 0, 30], period=2.0, atime=0.02, repeat=5, ref=DR_TOOL)
        while not check_force_condition(DR_AXIS_Z, max=40):
            pass
        release_compliance_ctrl()

        release()
        movej(JReady, vel=VELOCITY, acc=ACC)    

        rclpy.shutdown()
if __name__ == "__main__":
    main()
