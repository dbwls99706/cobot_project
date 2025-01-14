

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 300, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, trans,
            set_digital_output,
            set_tool, set_tcp,
            task_compliance_ctrl,
            set_desired_force,
            release_compliance_ctrl,
            check_force_condition,
            wait,
            DR_AXIS_Z,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL, 
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

    def move_cup(start_pos, end_pos):
        release()                                                         # 그리퍼 열림
        movel(start_pos, vel=VELOCITY, acc=ACC)                           # 잡을 컵의 상단으로 이동
        movel(zdown, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)      # 컵을 잡을 위치로 이동
        grip()                                                            # GRIP!!
        movel(zup, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)        # 컵을 다른 컵들로부터 빼기위해 z축으로 들어올림
        movel(xup, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)        # 컵이 이동하다 처음 위치에 존재하는 컵들을 치지 않기위해 앞으로 이동
        movel(end_pos, vel=VELOCITY, acc=ACC)                             # 컵을 놓을 위치의 상단으로 이동
        movel(zdown, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)      # 시간 단축위해 일정값 아래로 이동

        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])          # z방향으로 -30N의 힘으로 내리며 force condition 체크
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):                # 5N이상의 힘이 작용될 경우 종료
            pass
        release_compliance_ctrl() 

        release()                                                         # RELEASE!!!
        movel(end_pos, vel=VELOCITY, acc=ACC)                             # 놓은 컵이 흔들리지 않기 위해 놓은 위치의 상단으로 이동



    def turn_cup(start_pos, end_pos):
        release()                                                         # 그리퍼 열림( 열려있을 가능성)
        movel(start_pos, vel=100, acc=ACC)                                # 그리퍼를 잡을 위치의 상단으로 이동
        movel(zdown2, vel = 100, acc = ACC, mod = DR_MV_MOD_REL)          # 상대좌표를 사용해 컵을 잡을 위치로 이동
        grip()                                                            # 그리퍼 닫힘
        movel(zup, vel = 100, acc = ACC, mod = DR_MV_MOD_REL)             # 잡은 컵이 다른 물체에 걸리지 않도록 살짝 들어올림
        movej(turn, vel= 100, acc =ACC, mod = DR_MV_MOD_REL)              # Joint 6를 90도 씩 회전
        movej(turn, vel= 100, acc =ACC, mod = DR_MV_MOD_REL)              # 180도 회전 명령 시 오류가 생김
        movel(zup2, vel = 100, acc = ACC, mod = DR_MV_MOD_REL)            # 목표 위치까지 이동할 때 부딫히지 않기 위해
        movel(end_pos, vel=VELOCITY, acc=ACC)                             # end point로 이동

        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])          # 이동 후 안정적으로 놓기 위해 force condition
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        release_compliance_ctrl()
        release()                                                         # 컵 위에 닿으면 그리퍼 열림
        movel(down, vel = VELOCITY, acc = ACC, mod = DR_MV_MOD_REL)       # 초기 컵에 간섭하지 않기위해 그리퍼를 놓은 방향으로 제거


    JReady = [0, 0, 90, 0, 90, 0]
    zdown = [0, 0, -20, 0, 0, 0]
    zdown1 = posx(0, 0, -12, 0, 0, 0)
    zdown2 =[0, 0, -45, 0, 0, 0]
    JR = [0, 0, 0, 0, 0, 180]
    zup = [0, 0, 100, 0, 0, 0] # 들어올리기 
    zup2 = [0, 0, 50, 0, 0, 0]
    xup = [120, 0, 0, 0, 0, 0]
    down = [-100, -100, 0, 0, 0, 0]
    turn = [0, 0, 0, 0, 0, -90]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    cup1 = posx(272.375, -197.656, 200.4, 2.998, -179.319, 4.023) #시작 높이 (컵 11개)
    cup=[cup1]

    for i in range (9):
        prev_cup = posx(*cup[i])
        cup.append(trans(prev_cup, zdown1))

    cpos1_1 = posx(600.224, -81.234, 180.17, 89.978, -179.381, 90.876)      #1~6번(1층)좌표는 전부 get_current_pos로 구함
    cpos1_2 = posx(598.293, -4.537, 180.72, 54.443, -179.26, 55.234)        #2층 3층은 각 좌표값을 통해 계산
    cpos1_3 = posx(600.129, 72.453, 180.519, 63.009, -179.275, 63.303)
    cpos1_4 = posx(530.565, 40.163, 180.017, 55.163, -179.382, 55.7)
    cpos1_5 = posx(530.751, -42.885, 180.583, 54.077, -179.437, 54.73)
    cpos1_6 = posx(464.756, -4.085, 180.966, 49.847, -179.684, 50.417)
    cpos2_1 = [(cpos1_1[0]+cpos1_4[0])/2, (cpos1_1[1]+cpos1_2[1])/2, 274.966, 49.847, -179.684, 50.417]
    cpos2_2 = [(cpos1_2[0]+cpos1_4[0])/2, (cpos1_2[1]+cpos1_3[1])/2, 274.966, 49.847, -179.684, 50.417]
    cpos2_3 = [(cpos1_4[0]+cpos1_6[0])/2, -4.085, 274.966, 49.847, -179.684, 50.417]
    cpos3 = [554.264, -4.537, 295.815, 96.932, -179.649, 97.426]
    cpos = [cpos1_1, cpos1_2, cpos1_3, cpos1_4, cpos1_5, cpos1_6, cpos2_1, cpos2_2, cpos2_3, cpos3]
    last_cup = [281.457, -177.982, 115.014, 59.331, 93.271, 90.613]
    put_last_cup = [570.118, 15.017, 330.672, 61.703, 94.896, -89.788]

    while rclpy.ok():
        release()
        print("I'm Ready")
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        for i in range(10):
            movej(JReady, vel=VELOCITY, acc=ACC)
            print(f"{i+1}번 컵 옮기는 중")
            move_cup(cup[i], cpos[i])
        
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        print("11번 컵 옮기는 중")
        turn_cup(last_cup, put_last_cup)
        movej(JReady, vel=VELOCITY, acc=ACC)
        print("프로그램 종료")
        

        rclpy.shutdown()
if __name__ == "__main__":
    main()
