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

    def is_within_tolerance(pos_a, pos_b):
        tolerance = 3.0
        # x 좌표 차이 확인
        x_diff = abs(pos_a[0] - pos_b[0])
        # y 좌표 차이 확인
        y_diff = abs(pos_a[1] - pos_b[1])
        
        # x, y 모두 공차 내에 있는지 확인
        return x_diff <= tolerance and y_diff <= tolerance
    
    def find_first_not_10(blocks):
        """
        block_h에서 값이 10이 아닌 첫 번째 인덱스를 반환.
        없으면 None 반환.
        """
        for idx, value in enumerate(blocks):
            if value != 10:
                return idx
        return None

    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx(347.96, 94.11, 100.0, 10.62, -179.61, 11.19)
    pos2 = posx(400.54, 93.725, 100.0, 10.94, -179.45, 11.605)
    pos3 = posx(450.38, 93.34, 100.0, 11.26, -179.29, 12.02)
    pos4 = posx(346.92, 42.39, 100.0, 10.385, -179.35, 11.15)
    pos5 = posx(399.775, 42.46, 100.0, 12.105, -179.305, 12.8575)
    pos6 = posx(450.38, 42.53, 100.0, 13.825, -179.265, 14.565)
    pos7 = posx(345.88, -9.33, 100.0, 10.15, -179.08, 11.11)
    pos8 = posx(399.01, -8.805, 100.0, 13.27, -179.16, 14.11)
    pos9 = posx(450.38, -8.28, 100.0, 16.39, -179.24, 17.11)
    positions = [pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9]
    pos5_put = posx(549.775, 42.46, 100.0, 12.105, -179.305, 12.8575)
    zdown = [0, 0, -65, 0, 0, 0]
    zput = [0, 0, -50, 0, 0, 0] 
    zup = [0, 0, 65, 0, 0, 0]
    xup = [150, 0, 0, 0, 0, 0]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    return_sort = [[],[],[]]
    block_h = []
    emptynum = 0

    while rclpy.ok():
        release()
        print("I'm Ready")
        movej(JReady, vel=VELOCITY, acc=ACC)

        for i in range(9): #높이 측정
            grip()
            movel(positions[i], vel = VELOCITY, acc = ACC)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
            
            current_position = get_current_pos()
            print(f"{current_position[0][2]}")
            release_compliance_ctrl()
            movel(positions[i], vel = VELOCITY, acc = ACC)
            if(current_position[0][2] >= 60):
                return_sort[0].append(current_position[0])
                block_h.append(3)
                print(return_sort)
            elif(current_position[0][2] >= 50):
                return_sort[1].append(current_position[0])
                block_h.append(2)
                print(return_sort)
            elif(current_position[0][2] >= 40):
                return_sort[2].append(current_position[0])
                block_h.append(1)
                print(return_sort)

        for i in range(9):
            return_sort[i//3][i%3][2] = 100
            print(f"{return_sort[i//3][i%3]}")

        release()
        movel(return_sort[1][0], vel = VELOCITY, acc = ACC) #중간 높이 블록 옮기기
        movel(zdown, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)
        grip()
        movel(return_sort[1][0], vel = VELOCITY, acc = ACC)
        movel(pos5_put, vel = VELOCITY, acc = ACC)
        movel(zput, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)
        release()
        movel(zup, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)

        for i in range(9):
            if is_within_tolerance(return_sort[1][0], positions[i]):
                print(positions[i])
                print(f"empty num : {i}")
                emptynum = i
                block_h[i] = 0
                break
        
        for i in range(8):
            if(emptynum == 4):
                a = find_first_not_10(block_h)
                movel(positions[a],vel = VELOCITY, acc = ACC)
                movel(zdown, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)
                grip()
                movel(positions[a], vel = VELOCITY, acc = ACC)
                movel(positions[emptynum], vel = VELOCITY, acc = ACC)
                movel(zput, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)
                release()
                movel(positions[emptynum], vel = VELOCITY, acc = ACC)
                block_h[emptynum] = block_h[a]
                emptynum = a
                block_h[emptynum] = 0

            elif(emptynum >= 3 and emptynum < 6):
                a = block_h.index(2)
            elif(emptynum >= 6):
                a = block_h.index(3)
            else:
                a = block_h.index(1)    

            if(emptynum != 4):
                movel(positions[a],vel = VELOCITY, acc = ACC)
                movel(zdown, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)
                grip()
                movel(positions[a], vel = VELOCITY, acc = ACC)
                movel(positions[emptynum], vel = VELOCITY, acc = ACC)
                movel(zput, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)
                release()
                movel(positions[emptynum], vel = VELOCITY, acc = ACC)
                block_h[emptynum] = 10
                block_h[a] = 0
                emptynum = a

        movel(pos5_put, vel = VELOCITY, acc = ACC)
        movel(zdown, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)
        grip()
        movel(pos5_put, vel = VELOCITY, acc = ACC)
        movel(pos5, vel = VELOCITY, acc = ACC)
        movel(zput, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)
        release()
        movel(pos5, vel = VELOCITY, acc = ACC)




        rclpy.shutdown()
if __name__ == "__main__":
    main()
