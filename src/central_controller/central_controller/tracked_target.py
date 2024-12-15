class TrackedTarget:
    _id_counter = 0
    def __init__(self, pose, patrol_id):
        TrackedTarget._id_counter += 1
        self.id = TrackedTarget._id_counter
        self.pose:float = pose
        self.status = 0 # 0: unknown person, 1: known
        self.tracked_by = patrol_id # 감시당하고 있는 bot 주체

    def is_same_object(self, pose):
        difference = (self.pose[0] - pose[0])**2 + (self.pose[1] - pose[1])**2
        if difference < 25:
            return True
        else:
            return False
    
