"""独立命中核验：仅把指定装甲板碰撞体的物理接触记作装甲命中。"""

ARMOR_MODEL = "enemy_base_moving_armor"


class PressureSwitch:
    """板端触压开关：连续接触只产生一个上升沿，分离250ms后可再次触发。"""

    def __init__(self):
        self.last = {}
        self.sequence = {}

    def update(self, vehicle, stamp, pressed):
        if not pressed or stamp <= self.last.get(vehicle, -1):
            return None
        previous = self.last.get(vehicle, -1000000)
        self.last[vehicle] = stamp
        if stamp - previous <= 250000:
            return None
        self.sequence[vehicle] = self.sequence.get(vehicle, 0) + 1
        return {
            "schema": 1,
            "kind": "armor_pressure",
            "vehicle": vehicle,
            "sim_time_us": stamp,
            "sequence": self.sequence[vehicle],
            "pressed": True,
            "source": "SIM_ARMOR_CONTACT_SWITCH",
        }


def armor_contact(contacts):
    """匹配碰撞实体名称，不用无人机距基地中心的距离代替命中。"""
    for contact in contacts:
        for collision in (contact.collision1, contact.collision2):
            name = collision.name
            if ARMOR_MODEL in name and "armor_plate" in name:
                return True
    return False
