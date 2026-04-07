# 哨兵状态信息同步 (V1.1.0 0x020D)
# 裁判系统每秒1次主动推送哨兵当前状态，包括已购买的弹药血量、是否可复活及复活成本、当前姿态、能量机关激活状态等。哨兵决策层根据这些信息做出相应决策。

uint16 exchanged_projectile_allowance    # 成功兑换的允许发弹量
uint8 remote_projectile_exchange_count   # 远程兑换允许发弹量次数
uint8 remote_hp_exchange_count           # 远程兑换血量次数
bool can_free_respawn                    # 是否可确认免费复活
bool can_pay_respawn                     # 是否可兑换立即复活
uint16 respawn_gold_cost                 # 立即复活金币数
uint8 current_posture                    # 哨兵当前姿态
bool can_activate_rune                   # 能量机关是否可激活

# 哨兵姿态常量定义
uint8 POSTURE_ATTACK = 1
uint8 POSTURE_DEFENSE = 2
uint8 POSTURE_MOVE = 3
