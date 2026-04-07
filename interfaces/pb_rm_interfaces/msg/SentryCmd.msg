# 哨兵自主决策指令 (V1.1.0 0x0120)

bool confirm_respawn                     # 确认免费复活
bool confirm_pay_respawn                 # 确认付费复活
uint16 projectile_allowance_to_exchange  # 兑换发弹量
uint8 remote_projectile_request_count    # 远程兑换发弹量请求次数
uint8 remote_hp_request_count            # 远程兑换血量请求次数
uint8 posture_command                    # 切换姿态
bool activate_rune                       # 激活能量机关

uint8 POSTURE_ATTACK = 1                 # 进攻
uint8 POSTURE_DEFENSE = 2                # 防御
uint8 POSTURE_MOVE = 3                   # 移动
