"""独立命中核验：仅把指定装甲板碰撞体的物理接触记作装甲命中。"""

ARMOR_MODEL = "enemy_base_moving_armor"


def armor_contact(contacts):
    """匹配碰撞实体名称，不用无人机距基地中心的距离代替命中。"""
    for contact in contacts:
        for collision in (contact.collision1, contact.collision2):
            name = collision.name
            if ARMOR_MODEL in name and "armor_plate" in name:
                return True
    return False
