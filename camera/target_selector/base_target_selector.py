class BaseTargetSelector:
    def get_target_bbox(self, frame):
        """
        Return a bounding box for the target as (x, y, width, height).
        """
        raise NotImplementedError
